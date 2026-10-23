`timescale 1ns/1ps

// 模块名称：btb_trap_unit
// 功能与设计思路：
//   本模块实现高级控制逻辑：包含动态分支预测器 (BTB) 以及 RISC-V 机器模式异常与中断处理机制。
//   1. BTB 动态分支目标缓冲预测器：采用 32 项直接映射表项，每项包含 Tag、跳转目标 Target PC、有效位以及 2 位饱和状态机（2-bit Saturating Counter），在 IF 级实现 0 气泡推测取指。
//   2. 机器模式异常与中断特权控制系统：完整实现标准 M-Mode 控制状态寄存器（mstatus, mie, mip, mtvec, mscratch, mepc, mcause, mtval 等），支持 Direct 与 Vectored 向量表跳转，提供外部/软件/定时器三级中断优先级仲裁与硬件特权栈保护。

module btb_trap_unit #(
  parameter [31:0] RESET_MTVEC = 32'h00000100,  // 默认 mtvec 异常基址（复位初始向量入口）
  parameter integer BTB_ENTRIES = 32             // BTB 预测表项数（默认 32 项直接映射）
)(
  input  wire        clk,                       // 27MHz 系统同步主时钟
  input  wire        reset,                     // 全局同步高有效复位信号

  // 一、 动态分支目标缓冲预测器 (BTB) 接口
  // IF 级取指预测查找 (取指时钟沿同拍给出组合预测结果)
  input  wire [31:0] fetch_pc,                  // IF 级当前正在请求的指令 PC 地址
  output wire        btb_hit,                   // BTB 标签命中指示
  output wire        btb_take,                  // 预测跳转建议（命中且 2-bit 饱和计数器 >= 2）
  output wire [31:0] btb_target,                // 预测的跳转目标 PC 地址

  // EX 级分支预测训练更新
  input  wire        btb_update_en,             // 分支指令结案使能 (complete && !take_trap && is_branch)
  input  wire [31:0] btb_update_pc,             // 当前分支指令 PC (x_pc)
  input  wire        btb_update_jump,           // 分支真实跳转方向决策 (jump)
  input  wire [31:0] btb_update_target,         // 分支真实跳转目标 PC (target)
  input  wire        btb_update_mispredict,     // 分支预测失误脉冲 (branch_mispredict)

  // BTB 硬件性能计数器 (支持 CSR 与顶层输出)
  output wire [63:0] btb_hit_count,             // 预测成功累计计数
  output wire [63:0] btb_miss_count,            // 预测失误累计计数

  // 二、 RISC-V 异常与中断控制单元 (Trap / Interrupt / CSR) 接口
  // 外部物理中断输入
  input  wire        timer_irq,                 // MTIP 机器模式定时器中断 (来自 64 位周期比较器)
  input  wire        ext_irq,                   // MEIP 机器模式外部中断 (来自串口 RX 接收未决等)
  input  wire        soft_irq,                  // MSIP 机器模式软件中断 (来自 MMIO 软件写触发)

  // 流水线状态与中断仲裁握手
  input  wire        x_valid,                   // EX 级存在有效指令
  input  wire        complete,                  // EX 级指令本拍执行完毕
  input  wire        csr_access,                // 当前为 CSR 读写指令
  input  wire        do_mret,                   // 当前为 MRET 特权返回指令
  output wire        take_irq,                  // 处理器本拍响应异步中断高电平信号
  output wire [31:0] irq_target,                // 异步中断向量入口 (Direct 或 Vectored 向量表索引)

  // 精确异常陷入控制 (Trap Handling)
  input  wire        take_trap,                 // 处理器本拍陷入同步异常高电平 (complete && (exception || access_fault))
  input  wire [31:0] trap_in_pc,                // 出错指令 PC (x_pc)
  input  wire [31:0] trap_in_cause,             // 异常原因编码 (mcause)
  input  wire [31:0] trap_in_tval,              // 异常附加信息 (mtval，如未对齐访存物理地址)
  input  wire [31:0] irq_ret_pc,                // 中断保存后继返回 PC

  // 重定向地址导出
  output wire [31:0] mepc_out,                  // 当前 mepc 寄存器值 (供 MRET 特权返回重定向)
  output wire [31:0] mtvec_base,                // 当前异常向量 Direct 入口 {mtvec[31:2], 2'b00}

  // CSR 读写访问接口 (EX 级)
  input  wire [11:0] csr_addr,                  // 12 位 CSR 物理寄存器地址
  output wire [31:0] csr_rdata,                 // CSR 组合读取值
  output wire        csr_known,                 // CSR 地址已知合法标志 (0 触发非法指令异常)
  input  wire        csr_we,                    // CSR 寄存器写使能 (complete && csr_access && csr_write)
  input  wire [31:0] csr_wdata,                 // 经过位操作待写入 CSR 的新数据

  // 外部性能计数器导入 (供 CSR 读取)
  input  wire [63:0] cycles,                    // 处理器运行总时钟周期数
  input  wire [63:0] retired,                   // 成功提交指令总数

  // 跟踪调试接口 (Trace)
  output wire        trap_valid,                // 异常/中断发生脉冲指示
  output wire [31:0] trap_pc,                   // 发生异常/中断的指令 PC
  output wire [31:0] trap_cause                 // 异常/中断原因编码
);

  // 子模块例化 1：BTB 动态分支预测器
  btb #(
    .ENTRIES(BTB_ENTRIES)
  ) u_btb (
    .clk               (clk),
    .reset             (reset),
    .fetch_pc          (fetch_pc),
    .pred_hit          (btb_hit),
    .pred_take         (btb_take),
    .pred_target       (btb_target),
    .update_en         (btb_update_en),
    .update_pc         (btb_update_pc),
    .update_jump       (btb_update_jump),
    .update_target     (btb_update_target),
    .update_mispredict (btb_update_mispredict),
    .hit_count         (btb_hit_count),
    .miss_count        (btb_miss_count)
  );

  // 子模块例化 2：RISC-V 特权级异常中断控制单元
  rv32_trap_ctrl #(
    .RESET_MTVEC(RESET_MTVEC)
  ) u_trap_ctrl (
    .clk           (clk),
    .reset         (reset),
    .timer_irq     (timer_irq),
    .ext_irq       (ext_irq),
    .soft_irq      (soft_irq),
    .x_valid       (x_valid),
    .complete      (complete),
    .csr_access    (csr_access),
    .do_mret       (do_mret),
    .take_irq      (take_irq),
    .irq_target    (irq_target),
    .take_trap     (take_trap),
    .trap_in_pc    (trap_in_pc),
    .trap_in_cause (trap_in_cause),
    .trap_in_tval  (trap_in_tval),
    .irq_ret_pc    (irq_ret_pc),
    .mepc_out      (mepc_out),
    .mtvec_base    (mtvec_base),
    .csr_addr      (csr_addr),
    .csr_rdata     (csr_rdata),
    .csr_known     (csr_known),
    .csr_we        (csr_we),
    .csr_wdata     (csr_wdata),
    .cycles        (cycles),
    .retired       (retired),
    .btb_hit_count (btb_hit_count),
    .btb_miss_count(btb_miss_count),
    .trap_valid    (trap_valid),
    .trap_pc       (trap_pc),
    .trap_cause    (trap_cause)
  );

endmodule


// 子模块 1：btb —— 动态分支目标缓冲预测器 (Branch Target Buffer)
// 采用 32 项直接映射表结构，结合经典 2 位饱和计数器 (2-bit Saturating Counter)
// 状态转移图：
//   2'b00(强不跳转) <--(不跳)-- 2'b01(弱不跳转) <--(不跳)-- 2'b10(弱跳转) <--(不跳)-- 2'b11(强跳转)
//   2'b00(强不跳转) --(跳转)--> 2'b01(弱不跳转) --(跳转)--> 2'b10(弱跳转) --(跳转)--> 2'b11(强跳转)
module btb #(
  parameter integer ENTRIES = 32
)(
  input  wire        clk,                       // 27MHz 系统同步时钟
  input  wire        reset,                     // 全局同步复位

  // IF 级预测查找接口
  input  wire [31:0] fetch_pc,                  // 取指阶段 PC
  output wire        pred_hit,                  // 表项 Tag 命中且有效
  output wire        pred_take,                 // 建议跳转：命中且饱和计数器最高位为 1 (>= 2)
  output wire [31:0] pred_target,               // 缓冲的目标跳转地址

  // EX 级执行训练更新接口
  input  wire        update_en,                 // 分支指令结案更新使能
  input  wire [31:0] update_pc,                 // 分支指令 PC
  input  wire        update_jump,               // 实际决策：1 = 发生跳转, 0 = 顺序执行
  input  wire [31:0] update_target,             // 实际目标跳转 PC
  input  wire        update_mispredict,         // 是否发生预测失误

  // 性能统计计数器
  output reg  [63:0] hit_count,                 // 预测命中次数
  output reg  [63:0] miss_count                 // 预测失误次数
);

  // 【32 项直接映射表项】：
  // - btb_tag       : 25 位 Tag，对应 PC[31:7]
  // - btb_target_pc : 32 位目标地址
  // - btb_counter   : 2 位饱和计数器，默认初始化为 2'b01 (弱不跳)
  // - btb_valid     : 1 位表项有效位
  reg [31:0] btb_target_pc [0:ENTRIES-1];
  reg [24:0] btb_tag       [0:ENTRIES-1];
  reg [1:0]  btb_counter   [0:ENTRIES-1];
  reg        btb_valid     [0:ENTRIES-1];

  // IF 级直接映射寻址组合逻辑：PC[6:2] 作为 5 位表项索引，PC[31:7] 作为 25 位 Tag
  wire [4:0]  fetch_idx = fetch_pc[6:2];
  wire [24:0] fetch_tag = fetch_pc[31:7];

  assign pred_hit    = btb_valid[fetch_idx] && (btb_tag[fetch_idx] == fetch_tag);
  assign pred_take   = pred_hit && btb_counter[fetch_idx][1]; // 2'b10 或 2'b11 时推测跳转
  assign pred_target = btb_target_pc[fetch_idx];

  // EX 级更新索引与计算
  wire [4:0]  upd_idx = update_pc[6:2];
  wire [24:0] upd_tag = update_pc[31:7];

  integer i;
  always @(posedge clk) begin
    if (reset) begin
      hit_count  <= 64'd0;
      miss_count <= 64'd0;
      for (i = 0; i < ENTRIES; i = i + 1) begin
        btb_valid[i]     <= 1'b0;
        btb_tag[i]       <= 25'd0;
        btb_target_pc[i] <= 32'd0;
        btb_counter[i]   <= 2'b01;              // 初始偏向于保守弱不跳
      end
    end else if (update_en) begin
      // 训练更新：写入有效位、Tag 与最新计算的目标 PC
      btb_valid[upd_idx] <= 1'b1;
      btb_tag[upd_idx]   <= upd_tag;

      // 2 位饱和计数器状态转移
      if (update_jump) begin
        btb_target_pc[upd_idx] <= update_target;
        btb_counter[upd_idx]   <= (btb_counter[upd_idx] == 2'b11) ? 2'b11 : (btb_counter[upd_idx] + 2'b01);
      end else begin
        btb_counter[upd_idx]   <= (btb_counter[upd_idx] == 2'b00) ? 2'b00 : (btb_counter[upd_idx] - 2'b01);
      end

      // 实时维护命中与失误硬件计数器
      if (!update_mispredict) begin
        hit_count <= hit_count + 64'd1;
      end else begin
        miss_count <= miss_count + 64'd1;
      end
    end
  end

endmodule


// 子模块 2：rv32_trap_ctrl —— RISC-V 异常与中断特权控制单元
// 包含特权 CSR 寄存器堆、精确异常检测、三级中断仲裁、向量表生成与特权态栈
module rv32_trap_ctrl #(
  parameter [31:0] RESET_MTVEC = 32'h00000100
)(
  input  wire        clk,
  input  wire        reset,

  // 物理中断输入信号
  input  wire        timer_irq,                 // MTIP 定时器中断
  input  wire        ext_irq,                   // MEIP 外部中断
  input  wire        soft_irq,                  // MSIP 软件中断

  // 流水线状态与握手
  input  wire        x_valid,
  input  wire        complete,
  input  wire        csr_access,
  input  wire        do_mret,
  output wire        take_irq,                  // 响应中断脉冲
  output wire [31:0] irq_target,                // 中断重定向目标 PC

  // 异常陷入
  input  wire        take_trap,                 // 响应同步异常
  input  wire [31:0] trap_in_pc,                // 出错 PC
  input  wire [31:0] trap_in_cause,             // 异常原因
  input  wire [31:0] trap_in_tval,              // 异常附加值
  input  wire [31:0] irq_ret_pc,                // 中断返回 PC

  // 重定向地址与状态
  output wire [31:0] mepc_out,
  output wire [31:0] mtvec_base,

  // CSR 访问接口
  input  wire [11:0] csr_addr,
  output reg  [31:0] csr_rdata,
  output reg         csr_known,
  input  wire        csr_we,
  input  wire [31:0] csr_wdata,

  // 外部性能计数器导入
  input  wire [63:0] cycles,
  input  wire [63:0] retired,
  input  wire [63:0] btb_hit_count,
  input  wire [63:0] btb_miss_count,

  // 跟踪输出
  output reg         trap_valid,
  output reg  [31:0] trap_pc,
  output reg  [31:0] trap_cause
);

  // 【机器模式控制与状态寄存器堆 (M-Mode CSRs)】：
  // 严格依照 RISC-V Privileged Architecture 手册分配物理地址与位域
  reg [31:0] mtvec;                             // 0x305: 异常向量基址与工作模式 ([1:0]=00 Direct, 01 Vectored)
  reg [31:0] mepc;                              // 0x341: 异常/中断返回 PC 寄存器
  reg [31:0] mcause;                            // 0x342: 陷入原因编码寄存器 (最高位1=异步中断, 0=同步异常)
  reg [31:0] mtval;                             // 0x343: 异常附加值寄存器 (存放未对齐地址或非法指令机器码)
  reg [31:0] mscratch;                          // 0x340: 机器模式临时擦写寄存器 (供上下文保护暂存栈指针)
  reg [31:0] mstatus;                           // 0x300: 状态寄存器 (bit 7: MPIE, bit 3: MIE, bits[12:11]: MPP=2'b11)
  reg [31:0] mie;                               // 0x304: 中断使能寄存器 (bit 11: MEIE, bit 7: MTIE, bit 3: MSIE)
  reg        mip_soft_reg;                      // 0x344: 软件中断未决控制位

  assign mepc_out   = mepc;
  assign mtvec_base = {mtvec[31:2], 2'b00};

  // 中断信号清洗与悬起合成
  wire clean_timer_irq = (timer_irq === 1'b1);
  wire clean_ext_irq   = (ext_irq === 1'b1);
  wire clean_soft_irq  = (soft_irq === 1'b1);

  // 组装标准 mip 寄存器位图：bit 11 (MEIP), bit 7 (MTIP), bit 3 (MSIP)
  wire [31:0] mip_wire    = {20'b0, clean_ext_irq, 3'b0, clean_timer_irq, 3'b0, (clean_soft_irq | mip_soft_reg), 3'b0};
  wire [31:0] irq_pending = mip_wire & mie;      // 悬起且经 mie 使能放行的中断
  wire        irq_active  = mstatus[3] && (|irq_pending); // 全局中断 mstatus.MIE 打开且存在待决中断

  // 【三级中断优先级仲裁】：
  // 依照 RISC-V 标准：外部中断 (MEI, code 11) > 软件中断 (MSI, code 3) > 定时器中断 (MTI, code 7)
  wire [31:0] irq_code  = irq_pending[11] ? 32'd11 :
                          (irq_pending[3] ? 32'd3 : 32'd7);
  wire [31:0] irq_cause = {1'b1, 31'b0} | irq_code; // 最高位置 1 表示异步中断

  // 【中断向量目标计算】：
  // mtvec[1:0] == 2'b00：Direct 直接模式，所有中断统一跳转至基地址 BASE；
  // mtvec[1:0] == 2'b01：Vectored 向量模式，跳转至 BASE + 4 * cause 向量表索引。
  assign irq_target = (mtvec[1:0] == 2'b01) ?
                      ({mtvec[31:2], 2'b00} + (irq_code << 2)) :
                      {mtvec[31:2], 2'b00};

  // 中断响应条件：全局有效、无同步异常抢占、且在流水线完整指令边界响应
  assign take_irq = irq_active && !take_trap &&
                    (!x_valid || (complete && !csr_access && !do_mret));

  // 【CSR 多路选择组合读逻辑 (CSR Read Multiplexer)】
  always @* begin
    csr_known = 1'b1;
    case (csr_addr)
      12'h300: csr_rdata = mstatus;
      12'h304: csr_rdata = mie;
      12'h305: csr_rdata = mtvec;
      12'h340: csr_rdata = mscratch;
      12'h341: csr_rdata = mepc;
      12'h342: csr_rdata = mcause;
      12'h343: csr_rdata = mtval;
      12'h344: csr_rdata = mip_wire;
      // 性能计数器拆分读取 (高低 32 位无损映射)
      12'hC00: csr_rdata = cycles[31:0];
      12'hC80: csr_rdata = cycles[63:32];
      12'hC02: csr_rdata = retired[31:0];
      12'hC82: csr_rdata = retired[63:32];
      12'hC03: csr_rdata = btb_hit_count[31:0];
      12'hC83: csr_rdata = btb_hit_count[63:32];
      12'hC04: csr_rdata = btb_miss_count[31:0];
      12'hC84: csr_rdata = btb_miss_count[63:32];
      default: begin
        csr_rdata = 32'b0;
        csr_known = 1'b0;                       // 未知 CSR 地址触发非法指令异常
      end
    endcase
  end

  // 【时序逻辑：特权状态压栈/出栈与 CSR 软件写】
  always @(posedge clk) begin
    if (reset) begin
      mtvec        <= RESET_MTVEC;
      mepc         <= 32'd0;
      mcause       <= 32'd0;
      mtval        <= 32'd0;
      mscratch     <= 32'd0;
      mstatus      <= 32'h00001800;             // MPP = 2'b11 (强制锁定为 Machine 模式)
      mie          <= 32'd0;
      mip_soft_reg <= 1'b0;
      trap_valid   <= 1'b0;
      trap_pc      <= 32'd0;
      trap_cause   <= 32'd0;
    end else begin
      trap_valid <= take_trap || take_irq;

      // 1. 同步异常陷入 (Synchronous Trap)
      if (take_trap) begin
        mepc           <= trap_in_pc;           // 保存触发异常的精确指令 PC
        mcause         <= trap_in_cause;        // 记录异常原因码
        mtval          <= trap_in_tval;         // 记录异常错误地址或操作数
        trap_pc        <= trap_in_pc;
        trap_cause     <= trap_in_cause;
        // 硬件特权状态压栈：MPIE <= MIE, MIE <= 0（关中断防止嵌套破坏现场）, MPP <= Machine
        mstatus[7]     <= mstatus[3];
        mstatus[3]     <= 1'b0;
        mstatus[12:11] <= 2'b11;
      end

      // 2. 异步中断响应 (Asynchronous Interrupt)
      else if (take_irq) begin
        mepc           <= irq_ret_pc;           // 保存被打断的下一条待执行指令 PC
        mcause         <= irq_cause;            // 记录中断类型码
        mtval          <= 32'b0;
        trap_pc        <= irq_ret_pc;
        trap_cause     <= irq_cause;
        // 硬件特权状态压栈
        mstatus[7]     <= mstatus[3];
        mstatus[3]     <= 1'b0;
        mstatus[12:11] <= 2'b11;
      end

      // 3. MRET 特权指令返回
      else if (complete && do_mret) begin
        // 特权状态出栈恢复：MIE <= MPIE, MPIE <= 1, MPP <= Machine
        mstatus[3]     <= mstatus[7];
        mstatus[7]     <= 1'b1;
        mstatus[12:11] <= 2'b11;
      end

      // 4. CSR 寄存器软件写更新 (CSRRW / CSRRS / CSRRC)
      else if (csr_we) begin
        case (csr_addr)
          12'h300: mstatus      <= (csr_wdata & 32'h00000088) | 32'h00001800;
          12'h304: mie          <= csr_wdata & 32'h00000888;
          12'h305: mtvec        <= {csr_wdata[31:2], (csr_wdata[1:0] == 2'b01 ? 2'b01 : 2'b00)};
          12'h340: mscratch     <= csr_wdata;
          12'h341: mepc         <= {csr_wdata[31:2], 2'b00}; // 强制 4 字节对齐
          12'h342: mcause       <= csr_wdata;
          12'h343: mtval        <= csr_wdata;
          12'h344: mip_soft_reg <= csr_wdata[3];
          default: begin end
        endcase
      end
    end
  end

endmodule
