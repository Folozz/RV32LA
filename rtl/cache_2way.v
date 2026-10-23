`timescale 1ns/1ps

// 模块名称：cache_2way
// 功能与设计思路：
//   本模块实现 2 路组相联高速缓存存储器（容量 1KB，每路 512B，4 字 Cache Line）。
//   1. 命中检测与读写：并行比对 Way0 与 Way1 的 Tag，支持命中直接读写与伪 LRU 替换策略。
//   2. 缺失突发重填：Cache 缺失时向底层 RAM 发起 4 字突发读传输，完成整行填充后再响应 CPU。
//   3. 写穿与 Snoop 一致性：写操作直接透写至底层 RAM，同时支持硬件 Snoop 总线监听，当数据写命中时使对应缓存行失效，保证数据一致性。

module cache_2way #(
  parameter integer READ_ONLY = 0,               // 1 = 指令缓存(只读模式)；0 = 数据缓存(可读写)
  parameter integer RAM_BYTES = 65536            // 映射的片上物理主存空间大小 (64 KiB)
)(
  input  wire        clk,                        // 27MHz 系统同步主时钟
  input  wire        reset,                      // 全局同步高有效复位信号

  // 高阶控制与一致性维护接口
  input  wire        cancel,                     // 取消当前正在处理的事务（分支预测失误或异常冲刷时有效）
  input  wire        invalidate_all,             // 强制使全 Cache 行失效脉冲（响应 RISC-V FENCE.I 指令）
  input  wire        snoop_valid,                // Snoop 总线监听有效（由 D-Cache 发生写事务时向 I-Cache 发出）
  input  wire [31:0] snoop_addr,                 // Snoop 监听写地址

  // CPU 侧核心访存请求接口
  input  wire        req,                        // 访存请求使能
  input  wire        wr,                         // 读写方向：0 = 读, 1 = 写 (Store)
  input  wire [31:0] addr,                       // 32 位物理字节地址
  input  wire [31:0] wdata,                      // 32 位待写入数据
  input  wire [3:0]  wstrb,                      // 4 位字节写选通掩码
  output wire        ready,                      // 缓存事务完成应答握手信号（拉高代表 rdata 就绪或写入提交）
  output wire [31:0] rdata,                      // 读出的 32 位数据
  output reg         error,                      // 访问越界或未对齐错误（触发 CPU 精确异常）

  // 主存侧突发总线接口 (连接 soc_ram 双端口)
  output wire        mem_valid,                  // 发起底层 RAM 访问使能
  output wire        mem_write,                  // 底层 RAM 写使能
  output wire [31:0] mem_addr,                   // 突发地址（FILL 态下低 4 位按 beat 连续递增）
  output wire [31:0] mem_wdata,                  // 写往底层 RAM 的数据
  output wire [3:0]  mem_wstrb,                  // 写掩码
  input  wire        mem_ready,                  // 底层 RAM 应答就绪信号
  input  wire [31:0] mem_rdata,                  // 底层 RAM 突发读回数据

  // 硬件性能分析计数器（用于技术报告与 CoreMark 能效分析）
  output reg  [63:0] hits,                       // 缓存命中累计计数器
  output reg  [63:0] misses                      // 缓存缺失累计计数器
);

  // 状态机编码
  localparam IDLE  = 2'd0;                       // 空闲/等待并接收新请求
  localparam FILL  = 2'd1;                       // 缺失重填：4 字总线突发读取中
  localparam STORE = 2'd2;                       // 写穿提交：直写物理 RAM 中
  localparam RESP  = 2'd3;                       // 响应输出：保持稳定数据供 CPU 消费

  // 针对 64KiB 地址窗口，仅需地址 bits[15:9] 作为 7 位 Tag，大幅压缩比较器位宽以优化时序建立余量
  localparam integer TAG_BITS = (RAM_BYTES == 65536) ? 7 : 23;

  reg [1:0] state;

  // 【标签存储器与状态位】：
  // 32 组 × 2 路直接在寄存器阵列中管理，保证标签命中在 0 时延组合逻辑下即可检出
  reg [TAG_BITS-1:0] tag0 [0:31], tag1 [0:31];  // 路 0 与路 1 的 Tag 阵列
  reg                valid0 [0:31], valid1 [0:31];// 有效位
  reg                lru [0:31];                 // 伪 LRU 替换指针：0 表示上次命中/分配了路0，下次淘汰路1

  // 事务保存寄存器
  reg [31:0] saved_addr, saved_data;
  reg [3:0]  saved_strb;
  reg        saved_write, way;
  reg [1:0]  beat;                               // 4 字突发拍数计数器 (0, 1, 2, 3)
  reg        use_ram_data;
  reg [31:0] fill_data;

  // 【地址译码与命中判断组合逻辑】
  wire [4:0]          set_idx     = addr[8:4];
  wire [4:0]          saved_set   = saved_addr[8:4];
  wire [TAG_BITS-1:0] request_tag = addr[9 +: TAG_BITS];
  wire [TAG_BITS-1:0] saved_tag   = saved_addr[9 +: TAG_BITS];
  wire [TAG_BITS-1:0] snoop_tag   = snoop_addr[9 +: TAG_BITS];

  // 路 0 与路 1 命中判定
  wire hit0 = valid0[set_idx] && (tag0[set_idx] == request_tag);
  wire hit1 = valid1[set_idx] && (tag1[set_idx] == request_tag);

  // 牺牲路选择（Victim Selection）：若有空行优先填入空行；两路皆满时按 LRU 指针淘汰
  wire victim = !valid0[set_idx] ? 1'b0 : (!valid1[set_idx] ? 1'b1 : lru[set_idx]);

  // 地址合法性校验
  wire address_ok = (addr < RAM_BYTES) && (!READ_ONLY || (!wr && addr[1:0] == 0));

  // Snoop 地址命中范围判定
  wire snoop_ram     = snoop_valid && (snoop_addr < RAM_BYTES);
  wire snoop_active  = snoop_ram && (state != IDLE) && (snoop_addr[31:4] == saved_addr[31:4]);
  wire snoop_request = snoop_ram && (state == IDLE) && (snoop_addr[31:4] == addr[31:4]);

  // 异常或分支冲刷取消信号
  wire stop_transaction = reset || cancel || invalidate_all || snoop_active;
  wire accept_request   = (state == IDLE) && req && !stop_transaction && !snoop_request;

  // 外部总线握手与控制输出
  assign ready     = (state == RESP) && !stop_transaction;
  assign mem_valid = (state == FILL || state == STORE) && !stop_transaction;
  assign mem_write = (state == STORE);
  // 缺失重填时低 4 位拼接当前 beat，实现 4 字连续突发寻址
  assign mem_addr  = (state == FILL) ? {saved_addr[31:4], beat, 2'b00} : {saved_addr[31:2], 2'b00};
  assign mem_wdata = saved_data;
  assign mem_wstrb = saved_strb;

  // 【底层 DRM 数据阵列的例化与统一写口管理】：
  // 将突发读回的重填写入与 CPU 直写的更新写入，在进入 RAM 前统一合并为唯一的写使能/地址/数据
  wire fill_commit       = (state == FILL) && mem_ready && !stop_transaction;
  wire store_commit      = (state == STORE) && mem_ready && !stop_transaction && !READ_ONLY;
  wire [6:0] ram_write_addr = fill_commit ? {saved_set, beat} : saved_addr[8:2];
  wire [31:0] ram_write_data = fill_commit ? mem_rdata : saved_data;
  wire [3:0]  ram_write_mask = fill_commit ? 4'b1111 : (store_commit ? saved_strb : 4'b0000);

  wire [31:0] way0_data, way1_data;
  wire ram_read = accept_request && address_ok;

  // 实例化路 0 独立数据 RAM 模块（100% 诱导 DRM）
  rv32la_cache_data_way data_way0 (
    .clk     (clk),
    .rd_en   (ram_read),
    .rd_addr (addr[8:2]),
    .rd_data (way0_data),
    .wr_en   (way ? 4'b0000 : ram_write_mask),
    .wr_addr (ram_write_addr),
    .wr_data (ram_write_data)
  );

  // 实例化路 1 独立数据 RAM 模块（100% 诱导 DRM）
  rv32la_cache_data_way data_way1 (
    .clk     (clk),
    .rd_en   (ram_read),
    .rd_addr (addr[8:2]),
    .rd_data (way1_data),
    .wr_en   (way ? ram_write_mask : 4'b0000),
    .wr_addr (ram_write_addr),
    .wr_data (ram_write_data)
  );

  // 命中时由路选择开关输出对应 RAM 数据；重填时直通最后读回的数据
  assign rdata = use_ram_data ? (way ? way1_data : way0_data) : fill_data;

  // 【主控时序状态机】
  integer i;
  always @(posedge clk) begin
    if (reset) begin
      state        <= IDLE;
      error        <= 1'b0;
      hits         <= 64'd0;
      misses       <= 64'd0;
      saved_addr   <= 32'd0;
      saved_data   <= 32'd0;
      saved_strb   <= 4'd0;
      saved_write  <= 1'b0;
      way          <= 1'b0;
      beat         <= 2'd0;
      use_ram_data <= 1'b0;
      fill_data    <= 32'd0;
      for (i = 0; i < 32; i = i + 1) begin
        valid0[i] <= 1'b0;
        valid1[i] <= 1'b0;
        lru[i]    <= 1'b0;
      end
    end else begin
      // 分支冲刷取消、FENCE.I 全局失效或发生活跃写冲突时，无条件回归 IDLE
      if (cancel || invalidate_all || snoop_active) begin
        state <= IDLE;
      end else begin
        case (state)
          // 状态 0：IDLE（空闲检测与命中判定）
          IDLE: begin
            if (accept_request) begin
              saved_addr   <= addr;
              saved_data   <= wdata;
              saved_strb   <= wstrb;
              saved_write  <= wr;
              error        <= 1'b0;
              use_ram_data <= 1'b0;

              if (!address_ok) begin
                // 地址越界或非对齐取指：报 error，直接转入 RESP 通知流水线陷入异常
                error <= 1'b1;
                fill_data <= 32'd0;
                state <= RESP;
              end else if (hit0 || hit1) begin
                // 缓存命中！
                hits         <= hits + 64'd1;
                way          <= hit1;            // 标记命中哪一路
                lru[set_idx] <= !hit1;           // 更新 LRU：下次优先淘汰另一路
                use_ram_data <= 1'b1;
                // 若为写操作直穿物理 RAM (STORE)，若为读直接输出结果 (RESP)
                state        <= wr ? STORE : RESP;
              end else begin
                // 缓存缺失！
                misses <= misses + 64'd1;
                way    <= victim;                // 选定将被替换的牺牲路
                beat   <= 2'd0;
                state  <= FILL;                  // 启动 4 字总线突发重填状态机
                // 提前将目标路有效位置 0，防止重填期间读取脏数据
                if (victim) valid1[set_idx] <= 1'b0;
                else        valid0[set_idx] <= 1'b0;
              end
            end
          end

          // 状态 1：FILL（4 字突发重填）
          FILL: begin
            if (mem_ready) begin
              // 若当前拍刚好是 CPU 最初请求的那个字，锁存入 fill_data 供重填完成同拍消费
              if (beat == saved_addr[3:2])
                fill_data <= mem_rdata;

              if (beat == 2'd3) begin
                // 4 个字全部重填完毕，写入新 Tag，重新置 valid 为 1
                if (way) begin
                  tag1[saved_set]   <= saved_tag;
                  valid1[saved_set] <= 1'b1;
                end else begin
                  tag0[saved_set]   <= saved_tag;
                  valid0[saved_set] <= 1'b1;
                end
                lru[saved_set] <= !way;
                // 若本次事务本是 Store，重填后转去写穿；若为 Load，直接转入响应
                state <= saved_write ? STORE : RESP;
              end else begin
                beat <= beat + 2'd1;             // 突发拍数递增
              end
            end
          end

          // 状态 2：STORE（写穿物理内存）
          STORE: begin
            // 物理 RAM 确认写入完成后才转入 RESP，避免重叠写入导致时序冲突
            if (mem_ready)
              state <= RESP;
          end

          // 状态 3：RESP（就绪响应输出）
          RESP: begin
            // CPU 在本拍消费了该响应数据后，状态机安全回归 IDLE，准备接听下一笔请求
            if (req)
              state <= IDLE;
          end

          default: state <= IDLE;
        endcase
      end

      // 【Snoop 监听与全局失效时序维护】
      if (invalidate_all) begin
        // FENCE.I 指令执行：全缓存行有效位瞬间清零，强制后续指令全部重填
        for (i = 0; i < 32; i = i + 1) begin
          valid0[i] <= 1'b0;
          valid1[i] <= 1'b0;
        end
      end else if (snoop_ram) begin
        // D-Cache 写入监听：若写地址对应的 Set 中 Tag 匹配，原子将对应有效位置 0
        if (tag0[snoop_addr[8:4]] == snoop_tag) valid0[snoop_addr[8:4]] <= 1'b0;
        if (tag1[snoop_addr[8:4]] == snoop_tag) valid1[snoop_addr[8:4]] <= 1'b0;
      end
    end
  end

endmodule

// 子模块：rv32la_cache_data_way (单路 128×32位 标准同步专用块 RAM 模板)
// 功能说明：
//   本模块实现标准同步 RAM 数据阵列，综合推断为底层专用块 RAM (DRM)。
//   按 4 个 8 位字节通道并行排布，支持按字节写选通掩码。
module rv32la_cache_data_way (
  input  wire        clk,                       // 27MHz 同步时钟
  input  wire        rd_en,                     // 同步读使能
  input  wire [6:0]  rd_addr,                   // 7 位行读地址：32组 × 每行4字 = 128 项
  output wire [31:0] rd_data,                   // 32 位同步读出数据
  input  wire [3:0]  wr_en,                     // 4 位独立字节写使能
  input  wire [6:0]  wr_addr,                   // 7 位行写地址
  input  wire [31:0] wr_data                    // 32 位待写入数据
);

  genvar lane;
  generate
    for (lane = 0; lane < 4; lane = lane + 1) begin : byte_lane
      reg [7:0] data_mem [0:127];               // 128 深度 × 8 位单字节存储阵列
      reg [7:0] read_q;

      // 写端口：单地址、单使能，满足 BRAM 纯净写入规范
      always @(posedge clk) begin
        if (wr_en[lane])
          data_mem[wr_addr] <= wr_data[lane*8 +: 8];
      end

      // 读端口：标准 1 拍时钟同步输出，绝无组合逻辑旁路
      always @(posedge clk) begin
        if (rd_en)
          read_q <= data_mem[rd_addr];
      end

      assign rd_data[lane*8 +: 8] = read_q;
    end
  endgenerate

endmodule
