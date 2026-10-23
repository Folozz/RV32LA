`timescale 1ns/1ps

// 模块名称：rv32_soc
// 设计思路与系统架构：
//   本模块是 RV32LA 片上系统（SoC）核心总线互联枢纽。将自研的 RV32I 经典
//   三级流水线处理器核、2 路组相联双 Cache 子系统、64 KiB 原生块主存以及全套 MMIO 外设
//   （UART 串口、GPIO 交互、64 位高精度定时器/性能计数器、片上数字逻辑分析仪）高效融合在一起。
//
//   1. 【统一物理地址空间拓扑与分级译码】：
//      - 主存区间 (0x00000000 ~ 0x0000FFFF, 64 KiB)：代码与数据统一主存，由 I-Cache 和 D-Cache
//        分别通过独立的物理端口挂载，支持 4 字突发充填；
//      - MMIO 外设区间 (0x20000000 ~ 0x2000FFFF)：严格旁路 Cache（Bypass），所有读写均为直接、
//        精确单拍总线事务，彻底规避外设状态寄存器的缓存假读风险；
//      - 逻辑分析仪采样窗口 (0x30000000 ~ 0x30000FFF)：双口 RAM 独立物理映射，每个采样点占 4 字节
//        （低 8 位有效），支持 CPU 在采样完毕后进行高吞吐连续读出与 CRC 校验。
//
//   2. 【硬件 Snoop 监听总线保障强一致性】：
//      - 当 D-Cache 发生写穿物理主存时，SoC 内部硬件监听器自动截获写物理地址，并实时向 I-Cache 发出
//        Snoop Invalidation 废弃脉冲，使 I-Cache 中对应的行原子失效，从硬件底层彻底解决自修改代码一致性。
//
//   3. 【硬件三级中断生成逻辑】：
//      - 定时器中断 (MTIP)：由 64 位无撕裂硬件运行周期计数器与软件配置的 {timecmp_hi, timecmp_lo}
//        进行硬件比较，cycles 超过比较值时原子置位；
//      - 外部中断 (MEIP)：绑定串口接收就绪 `rx_full`，支持外部通信异步通知；
//      - 软件中断 (MSIP)：由 MMIO 寄存器 `mmio_msip[0]` 软件写 1 触发，支持跨任务协作。
//
//   4. 【双模信号探针路由器 (Dual-Mode Probe Router)】：
//      - source = 0：接入片内 32 位自由运行测试计数器，用于验证分频精确度与回传通信链路；
//      - source = 1：动态切换至 CPU 内部 8 路微架构探针（提交、停顿、前递、分支、BTB 命中、缓存缺失等），
//        实现芯片内部微架构“白盒级”可观测性。

module rv32_soc #(
  parameter integer CLK_HZ        = 27000000,    // 系统时钟频率：27 MHz
  parameter integer BAUD          = 115200,      // 串口波特率：115200 8N1
  parameter integer RAM_WORDS     = 16384,       // 64 KiB 对应 16384 个 32 位字
  parameter integer CAPTURE_DEPTH = 1024         // 逻辑分析仪最大硬件采样深度：1024 样本
)(
  input  wire        clk,                        // 27MHz 系统同步主时钟
  input  wire        reset,                      // 全局同步高有效复位信号
  input  wire        uart_rx,                    // 串口 RXD 物理输入
  output wire        uart_tx,                    // 串口 TXD 物理输出
  input  wire [7:0]  gpio_in,                    // 8 位 GPIO 输入（低 4 位接按键消抖锁存器）
  output reg  [7:0]  gpio_out,                   // 8 位 GPIO 输出（低 4 位驱动板载 LED）
  input  wire [7:0]  probes,                     // 外部备用探针输入

  // 仿真与验证跟踪接口
  output reg         sim_done,                   // 仿真完成指示（由软件写 EXIT 寄存器置位）
  output reg  [31:0] sim_code,                   // 退出状态码（0 代表全部断言通过）
  output wire        retire_valid,               // CPU 指令提交有效脉冲
  output wire [31:0] retire_pc, retire_insn,     // 提交指令 PC 与机器码
  output wire [4:0]  retire_rd,                  // 提交目标寄存器号
  output wire [31:0] retire_value,               // 写入寄存器的新值
  output wire        trap_valid,                 // 异常/中断陷入指示
  output wire [31:0] trap_pc, trap_cause         // 陷入指令 PC 与原因码
);

  // 【内部总线互连线网定义】：
  // i_* ：IF 取指通路（直连 I-Cache 请求端）
  // d_* ：EX 访存通路（经地址译码分流至 D-Cache 或 MMIO 外设）
  wire        i_en;
  wire [31:0] i_addr;
  wire [31:0] i_data;
  wire        i_error;
  wire        i_flush, i_invalidate;

  wire        d_valid, d_write;
  wire [31:0] d_addr, d_wdata;
  wire [3:0]  d_wstrb;
  wire        d_ready, d_error;
  reg         mmio_ready, mmio_error;
  reg  [31:0] bus_rdata;
  wire [31:0] d_rdata;

  // CPU 内部 64 位性能计数器导出
  wire [63:0] cycles, retired, stalls, branches, taken;
  wire [63:0] btb_hits, btb_misses;

  // 定时器与软件中断 MMIO 寄存器
  reg  [31:0] timecmp_lo, timecmp_hi, mmio_msip;
  reg         rx_full, rx_overrun;

  // 硬件中断源合成
  wire timer_irq = ({timecmp_hi, timecmp_lo} != 64'b0) && (cycles >= {timecmp_hi, timecmp_lo});
  wire ext_irq   = rx_full;                      // 串口收到新字节触发外部中断
  wire soft_irq  = mmio_msip[0];                 // 软件中断触发位

  // Cache 状态与命中统计信号
  wire        icache_ready, dc_ready, dc_error;
  wire [31:0] dc_data;
  wire [63:0] icache_hits, icache_misses, dcache_hits, dcache_misses;

  // 空间判定：地址小于 64KiB 属于片上 RAM，走 Cache；否则走 MMIO 外设
  wire ram_region = (d_addr < RAM_WORDS * 4);
  assign d_ready  = ram_region ? dc_ready : mmio_ready;
  assign d_error  = ram_region ? dc_error : mmio_error;

  // 8 通道微架构硬件探针线网
  wire [7:0] cpu_pipeline_probes;

  // 【一、处理器核心实例化 (rv32_core)】
  rv32_core cpu (
    .clk             (clk),
    .reset           (reset),
    .i_en            (i_en),
    .i_addr          (i_addr),
    .i_rdata         (i_data),
    .i_error         (i_error),
    .i_flush         (i_flush),
    .i_invalidate    (i_invalidate),
    .d_valid         (d_valid),
    .d_write         (d_write),
    .d_addr          (d_addr),
    .d_wdata         (d_wdata),
    .d_wstrb         (d_wstrb),
    .d_ready         (d_ready),
    .d_rdata         (d_rdata),
    .d_error         (d_error),
    .retire_valid    (retire_valid),
    .retire_pc       (retire_pc),
    .retire_insn     (retire_insn),
    .retire_rd       (retire_rd),
    .retire_value    (retire_value),
    .trap_valid      (trap_valid),
    .trap_pc         (trap_pc),
    .trap_cause      (trap_cause),
    .cycles          (cycles),
    .retired         (retired),
    .stall_cycles    (stalls),
    .branch_count    (branches),
    .taken_count     (taken),
    .timer_irq       (timer_irq),
    .ext_irq         (ext_irq),
    .soft_irq        (soft_irq),
    .i_ready         (icache_ready),
    .btb_hit_count   (btb_hits),
    .btb_miss_count  (btb_misses),
    .pipeline_probes (cpu_pipeline_probes)
  );

  // 【二、双 Cache 子系统实例化与 Snoop 强一致性连接】
  // 端口 A (im_*) 面向 I-Cache 突发充填；端口 B (dm_*) 面向 D-Cache 充填与写穿。
  // 当 D-Cache 发生写穿时 (ram_store = 1)，地址同步送往 I-Cache 执行 Snoop 失效。
  wire        im_valid, im_write, dm_valid, dm_write;
  wire [31:0] im_addr, dm_addr, im_wdata, dm_wdata;
  wire [3:0]  im_strb, dm_strb;
  wire        im_ready, dm_ready;
  wire [31:0] im_data, dm_data;

  wire ram_store = dm_valid && !dm_ready && dm_write && !reset;

  // 1 KB 2路组相联指令缓存 (I-Cache)
  cache_2way #(
    .READ_ONLY(1),
    .RAM_BYTES(RAM_WORDS * 4)
  ) icache (
    .clk            (clk),
    .reset          (reset),
    .cancel         (i_flush),
    .invalidate_all (i_invalidate),
    .snoop_valid    (ram_store),
    .snoop_addr     (dm_addr),
    .req            (i_en),
    .wr             (1'b0),
    .addr           (i_addr),
    .wdata          (32'b0),
    .wstrb          (4'b0),
    .ready          (icache_ready),
    .rdata          (i_data),
    .error          (i_error),
    .mem_valid      (im_valid),
    .mem_write      (im_write),
    .mem_addr       (im_addr),
    .mem_wdata      (im_wdata),
    .mem_wstrb      (im_strb),
    .mem_ready      (im_ready),
    .mem_rdata      (im_data),
    .hits           (icache_hits),
    .misses         (icache_misses)
  );

  // 1 KB 2路组相联数据缓存 (D-Cache)
  cache_2way #(
    .READ_ONLY(0),
    .RAM_BYTES(RAM_WORDS * 4)
  ) dcache (
    .clk            (clk),
    .reset          (reset),
    .cancel         (1'b0),
    .invalidate_all (1'b0),
    .snoop_valid    (1'b0),
    .snoop_addr     (32'b0),
    .req            (d_valid && ram_region),
    .wr             (d_write),
    .addr           (d_addr),
    .wdata          (d_wdata),
    .wstrb          (d_wstrb),
    .ready          (dc_ready),
    .rdata          (dc_data),
    .error          (dc_error),
    .mem_valid      (dm_valid),
    .mem_write      (dm_write),
    .mem_addr       (dm_addr),
    .mem_wdata      (dm_wdata),
    .mem_wstrb      (dm_strb),
    .mem_ready      (dm_ready),
    .mem_rdata      (dm_data),
    .hits           (dcache_hits),
    .misses         (dcache_misses)
  );

  // 【三、64 KiB 统一主存实例化 (soc_ram)】
  // 机器码由自研 scripts/build.py 自动内联固化，支持独立双端口并发读写
  soc_ram #(
    .RAM_WORDS(RAM_WORDS)
  ) u_ram (
    .clk      (clk),
    .reset    (reset),
    .im_valid (im_valid),
    .im_ready (im_ready),
    .im_addr  (im_addr),
    .im_data  (im_data),
    .dm_valid (dm_valid),
    .dm_ready (dm_ready),
    .dm_write (dm_write),
    .dm_strb  (dm_strb),
    .dm_addr  (dm_addr),
    .dm_wdata (dm_wdata),
    .dm_data  (dm_data)
  );

  // 【四、串口通信外设 (uart)】
  wire       tx_ready, rx_valid, rx_frame_error;
  wire [7:0] rx_data;
  reg        tx_start;
  reg  [7:0] tx_byte, rx_hold;

  uart #(
    .CLK_HZ(CLK_HZ),
    .BAUD  (BAUD)
  ) serial (
    .clk         (clk),
    .reset       (reset),
    .tx_start    (tx_start),
    .tx_data     (tx_byte),
    .tx_ready    (tx_ready),
    .tx          (uart_tx),
    .rx          (uart_rx),
    .rx_valid    (rx_valid),
    .rx_data     (rx_data),
    .frame_error (rx_frame_error)
  );

  // 【五、片上数字逻辑分析仪 (logic_analyzer)】
  reg  [31:0] la_div, la_length, la_trigger, la_source;
  reg         la_start, la_abort;
  reg         active_source;
  reg  [31:0] pattern;                          // 片内自由递增基准斜坡信号源
  reg  [7:0]  probes_meta, probes_sync;
  reg  [7:0]  gpio_meta, gpio_sync;

  wire        la_busy, la_done, la_error, la_triggered;
  wire [31:0] la_count, la_rdata;

  // 采样地址空间判断 (0x30000000 ~ 0x30000FFF, 4 KiB 窗口)
  wire la_region = (d_addr >= 32'h30000000 && d_addr < 32'h30000000 + CAPTURE_DEPTH * 4);
  wire la_read   = d_valid && !d_ready && !d_write && la_region;

  // 探针多路选择器：active_source=1 采 CPU 微架构探针，0 采片内计数器
  wire [7:0] sample_bus = active_source ? cpu_pipeline_probes : pattern[7:0];

  // 数据总线读返回多路选择器
  assign d_rdata = ram_region ? dc_data : (la_region ? la_rdata : bus_rdata);

  logic_analyzer #(
    .DEPTH(CAPTURE_DEPTH)
  ) sampler (
    .clk           (clk),
    .reset         (reset),
    .start         (la_start),
    .abort_capture (la_abort),
    .cfg_div       (la_div),
    .cfg_length    (la_length),
    .cfg_trigger   (la_trigger),
    .samples       (sample_bus),
    .busy          (la_busy),
    .done          (la_done),
    .error         (la_error),
    .triggered     (la_triggered),
    .count         (la_count),
    .rd_en         (la_read),
    .rd_index      ((d_addr - 32'h30000000) >> 2),
    .rd_data       (la_rdata)
  );

  // 【六、MMIO 外设地址译码与应答状态机】
  wire uart_write    = (d_addr == 32'h20000000 && d_write);
  // 总线接受条件：MMIO 请求有效且未就绪，写 UART 时必须等待上一帧发送完毕
  wire bus_accept    = d_valid && !ram_region && !d_ready && (!uart_write || (tx_ready && !tx_start));
  wire mmio_aligned  = (d_addr[1:0] == 2'b00);
  wire full_write    = (d_wstrb == 4'b1111);

  always @(posedge clk) begin
    if (reset) begin
      mmio_ready    <= 1'b0;
      mmio_error    <= 1'b0;
      bus_rdata     <= 32'd0;
      tx_start      <= 1'b0;
      tx_byte       <= 8'd0;
      rx_hold       <= 8'd0;
      rx_full       <= 1'b0;
      rx_overrun    <= 1'b0;
      gpio_out      <= 8'd0;
      gpio_meta     <= 8'd0;
      gpio_sync     <= 8'd0;
      probes_meta   <= 8'd0;
      probes_sync   <= 8'd0;
      sim_done      <= 1'b0;
      sim_code      <= 32'd0;
      active_source <= 1'b0;
      la_div        <= 32'd4;
      la_length     <= 32'd64;
      la_trigger    <= 32'd0;
      la_source     <= 32'd0;
      la_start      <= 1'b0;
      la_abort      <= 1'b0;
      pattern       <= 32'd0;
      timecmp_lo    <= 32'd0;
      timecmp_hi    <= 32'd0;
      mmio_msip     <= 32'd0;
    end else begin
      // 片内测试信号源每拍递增；输入信号两级同步打拍
      pattern     <= pattern + 32'd1;
      probes_meta <= probes;
      probes_sync <= probes_meta;
      gpio_meta   <= gpio_in;
      gpio_sync   <= gpio_meta;

      // 默认单拍清除脉冲
      mmio_ready <= 1'b0;
      mmio_error <= 1'b0;
      tx_start   <= 1'b0;
      la_start   <= 1'b0;
      la_abort   <= 1'b0;

      // 处理合法 MMIO 事务
      if (bus_accept) begin
        mmio_ready <= 1'b1;
        bus_rdata  <= 32'd0;

        if (la_region) begin
          // 逻辑分析仪采样缓冲窗口为只读区，禁止软件写入，且必须 4 字节对齐
          if (d_write || !mmio_aligned) mmio_error <= 1'b1;
        end else if (!mmio_aligned || (d_write && !full_write && d_addr != 32'h20000000)) begin
          mmio_error <= 1'b1;
        end else case (d_addr)
          // 0x20000000: UART_TX 发送寄存器 (写低 8 位启动硬件发送)
          32'h20000000: begin
            if (d_write && d_wstrb[0]) begin
              tx_start <= 1'b1;
              tx_byte  <= d_wdata[7:0];
            end
          end

          // 0x20000004: UART_STATUS 状态寄存器 (只读: bit0=TX就绪, bit2=RX有数据, bit3=RX溢出/错误)
          32'h20000004: begin
            bus_rdata <= {28'b0, rx_overrun, rx_full, 1'b0, (tx_ready && !tx_start)};
            if (d_write) mmio_error <= 1'b1;
          end

          // 0x20000008: UART_RX 接收数据寄存器 (只读: 读低 8 位并原子清除 rx_full 标志)
          32'h20000008: begin
            bus_rdata <= {24'b0, rx_hold};
            if (d_write) mmio_error <= 1'b1;
            else         rx_full   <= 1'b0;
          end

          // 0x2000000C: UART_INFO 串口信息与错误清除寄存器 (写 bit 0 清除错误标志)
          32'h2000000c: begin
            bus_rdata <= CLK_HZ / BAUD;
            if (d_write && d_wdata[0]) rx_overrun <= 1'b0;
          end

          // 0x20001000: GPIO_OUT 输出寄存器 (读写低 8 位，驱动板载 LED)
          32'h20001000: begin
            bus_rdata <= {24'b0, gpio_out};
            if (d_write) gpio_out <= d_wdata[7:0];
          end

          // 0x20001004: GPIO_IN 输入寄存器 (只读消抖锁存后的按键状态)
          32'h20001004: begin
            bus_rdata <= {24'b0, gpio_sync};
            if (d_write) mmio_error <= 1'b1;
          end

          // 0x20002000/04: CYCLES 64 位运行周期计数器 (低/高字只读)
          32'h20002000: begin bus_rdata <= cycles[31:0];  if (d_write) mmio_error <= 1'b1; end
          32'h20002004: begin bus_rdata <= cycles[63:32]; if (d_write) mmio_error <= 1'b1; end

          // 0x20002008/0C: RETIRED 64 位指令提交计数器 (低/高字只读)
          32'h20002008: begin bus_rdata <= retired[31:0];  if (d_write) mmio_error <= 1'b1; end
          32'h2000200c: begin bus_rdata <= retired[63:32]; if (d_write) mmio_error <= 1'b1; end

          // 0x20002010: STALLS 流水线数据相关停顿周期数 (只读)
          32'h20002010: begin bus_rdata <= stalls[31:0]; if (d_write) mmio_error <= 1'b1; end

          // 0x20002014: BRANCHES 条件分支指令总数 (只读)
          32'h20002014: begin bus_rdata <= branches[31:0]; if (d_write) mmio_error <= 1'b1; end

          // 0x20002018: TAKEN 条件分支成立次数 (只读)
          32'h20002018: begin bus_rdata <= taken[31:0]; if (d_write) mmio_error <= 1'b1; end

          // 0x2000201C: CLK_HZ 系统主频配置值 (只读，返回 27000000)
          32'h2000201c: begin bus_rdata <= CLK_HZ; if (d_write) mmio_error <= 1'b1; end

          // 0x20002020/24: TIMECMP 64 位定时器比较寄存器 (读写)
          32'h20002020: begin bus_rdata <= timecmp_lo; if (d_write) timecmp_lo <= d_wdata; end
          32'h20002024: begin bus_rdata <= timecmp_hi; if (d_write) timecmp_hi <= d_wdata; end

          // 0x20002028: MSIP 软件中断控制寄存器 (读写 bit 0)
          32'h20002028: begin bus_rdata <= mmio_msip; if (d_write) mmio_msip <= d_wdata; end

          // 0x20002030~3C: I-Cache 命中与缺失 64 位计数器 (只读)
          32'h20002030: begin bus_rdata <= icache_hits[31:0];   if (d_write) mmio_error <= 1'b1; end
          32'h20002034: begin bus_rdata <= icache_hits[63:32];  if (d_write) mmio_error <= 1'b1; end
          32'h20002038: begin bus_rdata <= icache_misses[31:0];  if (d_write) mmio_error <= 1'b1; end
          32'h2000203c: begin bus_rdata <= icache_misses[63:32]; if (d_write) mmio_error <= 1'b1; end

          // 0x20002040~4C: D-Cache 命中与缺失 64 位计数器 (只读)
          32'h20002040: begin bus_rdata <= dcache_hits[31:0];   if (d_write) mmio_error <= 1'b1; end
          32'h20002044: begin bus_rdata <= dcache_hits[63:32];  if (d_write) mmio_error <= 1'b1; end
          32'h20002048: begin bus_rdata <= dcache_misses[31:0];  if (d_write) mmio_error <= 1'b1; end
          32'h2000204c: begin bus_rdata <= dcache_misses[63:32]; if (d_write) mmio_error <= 1'b1; end

          // 0x20002050~5C: BTB 分支预测命中与失误 64 位计数器 (只读)
          32'h20002050: begin bus_rdata <= btb_hits[31:0];   if (d_write) mmio_error <= 1'b1; end
          32'h20002054: begin bus_rdata <= btb_hits[63:32];  if (d_write) mmio_error <= 1'b1; end
          32'h20002058: begin bus_rdata <= btb_misses[31:0];  if (d_write) mmio_error <= 1'b1; end
          32'h2000205c: begin bus_rdata <= btb_misses[63:32]; if (d_write) mmio_error <= 1'b1; end

          // 0x20003000: CAP_CONTROL 采样控制寄存器 (写 bit 0 启动, bit 1 取消)
          32'h20003000: if (d_write) begin
            la_start <= d_wdata[0];
            la_abort <= d_wdata[1];
            if (d_wdata[0] && !la_busy) active_source <= la_source[0];
          end

          // 0x20003004: CAP_STATUS 采样状态寄存器 (只读: bit0=忙, bit1=完成, bit2=错误, bit3=已触发)
          32'h20003004: begin
            bus_rdata <= {28'b0, la_triggered, la_error, la_done, la_busy};
            if (d_write) mmio_error <= 1'b1;
          end

          // 0x20003008: CAP_DIV 采样分频系数寄存器 (读写，1~65535)
          32'h20003008: begin bus_rdata <= la_div; if (d_write) la_div <= d_wdata; end

          // 0x2000300C: CAP_LENGTH 采样样本点数寄存器 (读写，1~1024)
          32'h2000300c: begin bus_rdata <= la_length; if (d_write) la_length <= d_wdata; end

          // 0x20003010: CAP_TRIGGER 触发模式配置寄存器 (读写)
          32'h20003010: begin bus_rdata <= la_trigger; if (d_write) la_trigger <= d_wdata; end

          // 0x20003014: CAP_COUNT 实际已采点数 (只读)
          32'h20003014: begin bus_rdata <= la_count; if (d_write) mmio_error <= 1'b1; end

          // 0x20003018: CAP_SOURCE 采样源选择 (读写: bit0=0 片内测试计数器，bit0=1 CPU 微架构探针)
          32'h20003018: begin bus_rdata <= la_source; if (d_write) la_source <= d_wdata; end

          // 0x2000F000: EXIT 仿真/测试结束寄存器 (写 0 成功，写非 0 失败并锁存退出码)
          32'h2000f000: begin
            if (d_write) begin
              sim_done <= 1'b1;
              sim_code <= d_wdata;
            end
          end

          default: mmio_error <= 1'b1;
        endcase
      end

      // 【串口接收字节写入与溢出裁决】：
      // 新数据到达时具有最高优先级：若原先存在未被 CPU 取走的数据且本拍没有读取，
      // 硬件自动置位 rx_overrun，提示上层发生硬件溢出。
      if (rx_valid) begin
        if (rx_full && !(bus_accept && !d_write && d_addr == 32'h20000008))
          rx_overrun <= 1'b1;
        rx_hold <= rx_data;
        rx_full <= 1'b1;
      end
      if (rx_frame_error)
        rx_overrun <= 1'b1;
    end
  end

endmodule
