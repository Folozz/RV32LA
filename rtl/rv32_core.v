`timescale 1ns/1ps
// 模块名称：rv32_core
// 功能描述：顺序执行、单发射、RV32I 基础整数指令集 3 级流水线处理器核
// 适用标准：RISC-V 非特权规范 (RV32I) 与 机器模式特权规范子集 (M-Mode)
//
// 流水线结构划分：
//   1. IF (Instruction Fetch, 取指级):
//      - 产生取指地址 fetch_pc，向带完成握手的指令缓存发起读取请求 (i_en, i_addr)；
//      - 记录当前 RAM 正在读出的 PC 与有效标记 (f_pc, f_valid)。
//   2. EX (Execute, 译码/执行/访存级):
//      - 提取指令字段 (op, f3, f7, rs1, rs2, rd) 并符号扩展各型立即数；
//      - WB->EX 数据前递 (Bypass)，彻底消除 ALU 指令间 RAW (写后读) 数据冲突；
//      - 组合 ALU 运算、分支跳转判断与目标地址计算；
//      - 数据总线访问控制 (d_valid, d_write, d_addr, d_wdata, d_wstrb)；
//      - 硬件精确异常检查 (对齐、非法指令、取指/总线访问错误)。
//   3. WB (Write Back, 写回/退休级):
//      - 在时钟沿将 EX 计算结果写入通用寄存器堆 (regs[rd])；
//      - 硬件强制保证 x0 恒等于 0；
//      - 驱动退休跟踪接口 (retire_*) 与 64 位性能计数器。

module rv32_core #(
  // 核心参数配置
  parameter [31:0] RESET_PC    = 32'h00000000, // 复位后首条指令的获取地址 (PC 起始点)
  parameter [31:0] RESET_MTVEC = 32'h00000100  // 软件改写 mtvec 前的默认 Trap 中断/异常向量入口基址
)(
  // 全局时钟与同步复位
  input  wire        clk,                      // 主时钟信号，上升沿触发内部状态更新
  input  wire        reset,                    // 同步高电平有效复位信号

  // 指令总线接口 (IF 级与同步片上 RAM 连接)
  // 时序关系：在 clk_N 发起 i_en & i_addr，clk_N+1 返回稳定的 i_rdata & i_error
  output wire        i_en,                     // 取指使能信号 (高电平表示发起读取)
  output wire [31:0] i_addr,                   // 32 位取指字节地址 (通常 4 字节对齐)
  input  wire [31:0] i_rdata,                  // 同步 RAM 返回的 32 位指令机器码
  input  wire        i_error,                  // 取指错误标志 (越界或未对齐，触发 Instruction Access Fault)

  // 数据总线接口 (EX 级访存：Load / Store 与外设 MMIO)
  // 握手协议：d_valid 拉高并保持稳定，直到 d_ready == 1 握手完成；读写均为单笔同步事务
  output wire        d_valid,                  // 数据访问请求有效 (保持高电平直至 d_ready)
  output wire        d_write,                  // 访问方向：1 = 写入 (Store)，0 = 读取 (Load)
  output wire [31:0] d_addr,                   // 32 位访存字节物理地址
  output wire [31:0] d_wdata,                  // 32 位待写入数据 (已按地址低 2 位完成字节对齐移位)
  output wire [3:0]  d_wstrb,                  // 4 位字节写使能掩码 (每个 bit 对应 32 位字中的 1 个字节)
  input  wire        d_ready,                  // 数据从设备就绪标志 (高电平代表本次读/写事务已完成)
  input  wire [31:0] d_rdata,                  // 从设备读回的 32 位对齐数据
  input  wire        d_error,                  // 数据总线访问错误 (仅在 d_ready 为高时有效，触发 Load/Store 错误)

  // 指令退休跟踪接口 (Retirement Trace, 用于仿真比对与硬件调试)
  output reg         retire_valid,             // 指令成功提交高脉冲 (1 拍有效)
  output reg  [31:0] retire_pc,                // 当前提交指令的 PC
  output reg  [31:0] retire_insn,              // 当前提交指令的 32 位机器码
  output reg  [4:0]  retire_rd,                // 当前提交指令的目的寄存器号 (不写或写 x0 时输出 0)
  output reg  [31:0] retire_value,             // 当前提交指令写入目的寄存器的数据值

  // 异常跟踪接口 (Trap Trace, 由 btb_trap_unit 驱动)
  output wire        trap_valid,               // 异常触发高脉冲 (1 拍有效)
  output wire [31:0] trap_pc,                  // 触发异常的指令 PC (存入 mepc)
  output wire [31:0] trap_cause,               // 异常原因编码 (存入 mcause)

  // 64 位硬件性能计数器 (可由软件差值计算 CPI 与 CoreMark 性能)
  output reg  [63:0] cycles,                   // 处理器运行的总时钟周期数
  output reg  [63:0] retired,                  // 成功执行并退休的指令总数
  output reg  [63:0] stall_cycles,             // EX 级因等待访存 (d_ready 等) 停顿的周期数
  output reg  [63:0] branch_count,             // 成功执行的条件分支指令总数
  output reg  [63:0] taken_count,              // 条件分支成立 (发生跳转) 的次数

  // 中断与取指等待扩展接口 (高阶控制逻辑与存储优化)
  input  wire        timer_irq,                // 机器模式定时器中断输入 (MTIP, 高电平有效)
  input  wire        ext_irq,                  // 机器模式外部中断输入 (MEIP, 高电平有效)
  input  wire        soft_irq,                 // 机器模式软件中断输入 (MSIP, 高电平有效)
  input  wire        i_ready,                  // 指令总线就绪信号 (I-Cache 缺失等待，1=就绪, 0=等待)
  output wire [63:0] btb_hit_count,            // BTB 预测成功次数 (由 btb_trap_unit 驱动)
  output wire [63:0] btb_miss_count,           // BTB 预测失败次数 (由 btb_trap_unit 驱动)
  output wire i_flush, // 重定向时取消尚未消费的取指事务
  output wire i_invalidate, // FENCE.I 使整个指令缓存失效
  output wire [7:0] pipeline_probes // 8通道流水线微架构观测探针 (供片上逻辑分析仪)
);

  // 【第一部分】通用寄存器堆与流水线状态寄存器
  // 32 个 32 位 RISC-V 通用整数寄存器；x0 在硬件上强制读恒为 0，写被丢弃
  reg [31:0] regs [0:31];

  // IF 级状态
  reg [31:0] fetch_pc;       // 下一拍准备向 RAM 请求的指令地址
  reg [31:0] f_pc;           // IF 缓冲内已完成取指的 PC
  reg [31:0] f_insn;
  reg f_error;
  reg        f_valid;        // IF 级是否存在有效指令待交给 EX 级

  // EX 级状态
  reg        x_valid;        // EX 级当前指令有效位
  reg        x_fetch_error;  // 取指阶段传递过来的总线/对齐错误标志
  reg [31:0] x_pc;           // EX 级当前指令的 PC
  reg [31:0] x_insn;         // EX 级当前指令的 32 位机器码

  // WB 级状态
  reg        w_valid;        // WB 级提交有效位 (仅在 EX 正常完成且无异常时置 1)
  reg        w_we;           // WB 级通用寄存器写使能意图
  reg [4:0]  w_rd;           // WB 级写入的目的寄存器编号
  reg [31:0] w_value;        // WB 级写回数据 (ALU 计算结果、Load 数据或 CSR 读值)
  reg [31:0] w_pc;           // WB 级指令的 PC (用于退休跟踪)
  reg [31:0] w_insn;         // WB 级指令机器码 (用于退休跟踪)

  // IF/EX 级分支预测流水线信号
  reg        f_pred_taken;
  reg [31:0] f_pred_target;
  reg        x_pred_taken;
  reg [31:0] x_pred_target;

  // 【第二部分】指令字段提取、数据前递 (Forwarding) 与立即数符号扩展

  // RISC-V 32 位标准指令格式字段拆解
  wire [6:0] op  = x_insn[6:0];                // 操作码 (Opcode)
  wire [2:0] f3  = x_insn[14:12];              // 3 位功能码 (funct3)
  wire [6:0] f7  = x_insn[31:25];              // 7 位功能码 (funct7)
  wire [4:0] rs1 = x_insn[19:15];              // 源操作数寄存器 1 编号
  wire [4:0] rs2 = x_insn[24:20];              // 源操作数寄存器 2 编号
  wire [4:0] rd  = x_insn[11:7];               // 目的寄存器编号

  // WB -> EX 数据前递
  // rs1==0 输出 0
  // 若 w_rd==rs1 && w_valid && w_we 前递 w_value，否则读 regs[rs1/2]
  wire [31:0] a = (rs1 == 0) ? 32'b0 :
                  ((w_valid && w_we && w_rd == rs1) ? w_value : regs[rs1]);
  wire [31:0] b = (rs2 == 0) ? 32'b0 :
                  ((w_valid && w_we && w_rd == rs2) ? w_value : regs[rs2]);

  // 各类指令立即数格式的符号扩展
  // I 型立即数 (用于计算、Load、JALR 等): insn[31:20] 符号扩展到 32 位
  wire [31:0] imm_i = {{20{x_insn[31]}}, x_insn[31:20]};
  // S 型立即数 (用于 Store): insn[31:25] 与 insn[11:7] 拼接后符号扩展
  wire [31:0] imm_s = {{20{x_insn[31]}}, x_insn[31:25], x_insn[11:7]};
  // B 型立即数 (用于条件分支): 打散拼接，最低位补 0 (2 字节对齐) 后符号扩展，偶数地址
  wire [31:0] imm_b = {{19{x_insn[31]}}, x_insn[31], x_insn[7], x_insn[30:25], x_insn[11:8], 1'b0};
  // J 型立即数 (用于 JAL): 打散拼接，最低位补 0 后符号扩展
  wire [31:0] imm_j = {{11{x_insn[31]}}, x_insn[31], x_insn[19:12], x_insn[20], x_insn[30:21], 1'b0};

  // 组合译码产生的内部控制与数据信号
  reg        legal;          // 当前指令机器码是否合法 (0 会触发非法指令异常)
  reg        regwrite;       // 写通用寄存器使能意图 (在发生异常时会被硬件拦截)
  reg        is_load;        // 标识当前为 Load 指令
  reg        is_store;       // 标识当前为 Store 指令
  reg        is_branch;      // 标识当前为条件分支指令 (用于分支统计)
  reg        jump;           // 分支成立或无条件跳转使能
  reg        do_mret;        // 标识当前正在执行 MRET 指令

  reg [31:0] result;         // EX 阶段计算结果 (用于写回寄存器)
  reg [31:0] address;        // 访存有效字节地址 (rs1 + offset)
  reg [31:0] target;         // 跳转/分支的目标 PC
  reg [31:0] shifted_load;   // 访存读回数据按地址低 2 位移位后的对齐字
  reg [3:0]  strobes;        // 访存写使能字节掩码
  reg [31:0] store_data;     // 访存写数据 (按字节偏移移位后)

  // 本地异常指示
  reg        exception;      // EX 级组合检测出异常 (对齐错误、非法指令等)
  reg [31:0] cause;          // 异常原因代码 (对应 mcause)
  reg [31:0] tval;           // 异常附加值 (对应 mtval)

  // CSR 操作内部信号
  reg        csr_access;     // 是否访问 CSR
  reg        csr_write;      // 是否对 CSR 执行写入操作
  wire       csr_known;      // CSR 地址是否在已知合法支持列表中 (由 btb_trap_unit 判定)
  wire [31:0] csr_old;       // 读取出的 CSR 原始值 (由 btb_trap_unit 组合输出)
  reg [31:0] csr_new;        // 经过位操作后准备写入 CSR 的新值
  reg [31:0] csr_source;     // CSR 操作源操作数 (来自 rs1 寄存器或 5 位立即数 zimm)
  wire [11:0] csr_addr = x_insn[31:20]; // 12 位 CSR 寻址地址

  // 【第三部分】流水线完成、停顿、冲刷与总线握手控制
  // 是否为访存指令 (Load 或 Store)
  wire mem_op = is_load || is_store;

  // EX 级指令完成判断 (complete):
  // 1. 本指令自身发生异常 (立即结案，不需要发总线等待)；
  // 2. 非访存指令 (运算/分支等单周期完成)；
  // 3. 访存指令收到外部数据总线的 d_ready 完成握手。
  wire complete = x_valid && (exception || !mem_op || d_ready);

  // 外部总线访问错误 (Access Fault): 仅在实际访存且收到 d_ready 响应并置 d_error 时采纳
  wire access_fault = x_valid && mem_op && !exception && d_ready && d_error;

  // 中断输入清洗与等待信号
  wire clean_i_ready   = (i_ready !== 1'b0);

  // 是否陷入异常处理 (Trap): 指令完成时出现本地异常或外部总线访问故障
  wire take_trap = complete && (exception || access_fault);

  // 分支预测执行校验与重定向判定
  wire branch_taken    = jump;
  wire [31:0] actual_target = branch_taken ? target : (x_pc + 32'd4);
  wire pred_matched    = (x_pred_taken == branch_taken) && (!branch_taken || (x_pred_target == target));
  wire branch_mispredict = is_branch && !pred_matched;
  wire is_jump_inst    = (op == 7'h6f || op == 7'h67);

  // 高级控制逻辑单元 (btb_trap_unit) 互联信号声明
  wire        btb_hit;
  wire        btb_take;
  wire [31:0] btb_target;
  wire        take_irq;
  wire [31:0] irq_target;
  wire [31:0] mepc_out;
  wire [31:0] mtvec_base;

  // 流水线 PC 重定向控制 (redirect):
  // 1. 同步异常陷入 (take_trap)
  // 2. 异步中断响应 (take_irq)
  // 3. 特权返回 (complete && do_mret)
  // 4. 分支预测失败 (complete && branch_mispredict)
  // 5. 无条件跳转执行 (complete && is_jump_inst && jump)
  wire fence_i = (op == 7'h0f && f3 == 3'b001);
  // 自修改代码可能使 BTB 保留旧分支；非分支被预测为跳转也必须纠正。
  wire stale_prediction = x_pred_taken && !is_branch && !is_jump_inst;
  wire redirect = take_trap || take_irq || (complete &&
      (do_mret || fence_i || stale_prediction || branch_mispredict || (is_jump_inst && jump)));
  assign i_flush = redirect;
  assign i_invalidate = complete && !take_trap && fence_i;

  // 重定向目标地址 (由 btb_trap_unit 导出 mtvec_base, irq_target, mepc_out)
  wire [31:0] redirect_target =
    take_trap ? mtvec_base :
    take_irq  ? irq_target :
    do_mret   ? mepc_out :
    ((fence_i || stale_prediction) ? x_pc + 32'd4 : (is_branch ? actual_target : target));

  // 中断保存的返回地址 (mepc)
  // 中断在已完成指令之后进入；保存体系结构后继PC，而不是预测器的猜测值。
  wire [31:0] irq_ret_pc = complete ?
      ((is_branch || is_jump_inst) ? actual_target : x_pc + 32'd4) :
      (f_valid ? f_pc : fetch_pc);

  // EX 级就绪信号 (x_ready): EX 级为空或当前指令已顺利完成，允许 IF 级灌入下一条指令
  wire x_ready = !x_valid || complete;

  // 指令总线接口驱动
  // IF缓冲有空位时发出请求；缺失时地址保持，ready返回后才推进PC
  assign i_en = (!f_valid || x_ready) && !redirect && !reset;
  assign i_addr = fetch_pc;

  // 数据总线接口驱动
  // 安全防护：若指令自身已发现未对齐等异常，立即屏蔽 d_valid，防止错误写坏内存/外设
  assign d_valid = x_valid && mem_op && !exception && !reset;
  assign d_write = is_store;
  assign d_addr  = address;
  assign d_wdata = store_data;
  assign d_wstrb = strobes;

  // 8 通道片上微架构观测探针 (供片上逻辑分析仪与性能分析)
  // Bit 0: retire_valid      —— 指令退休/提交有效脉冲 (1 拍有效，指示 1 CPI 吞吐)
  // Bit 1: pipeline_stall    —— 流水线停顿 (EX 级等待访存数据就绪或 MMIO 响应)
  // Bit 2: fwd_active        —— 旁路前递激活 (WB->EX RAW 前递生效，消除写后读气泡)
  // Bit 3: is_branch         —— 当前执行条件分支指令 (BEQ/BNE/BLT 等)
  // Bit 4: jump              —— 分支条件成立或发生跳转
  // Bit 5: btb_take          —— BTB 分支预测器预测命中并跳转 (IF 级 0 气泡重定向)
  // Bit 6: branch_mispredict —— BTB 预测失误 (EX 级发现预测错误，触发 1 周期冲刷)
  // Bit 7: icache_miss       —— I-Cache 缺失等待 (I-Cache 正在进行 4 字 Burst 重填)
  wire fwd_a = (w_valid && w_we && (w_rd != 5'd0) && (w_rd == rs1));
  wire fwd_b = (w_valid && w_we && (w_rd != 5'd0) && (w_rd == rs2));
  wire fwd_active = fwd_a || fwd_b;
  wire pipeline_stall = x_valid && !complete;

  assign pipeline_probes = {
    !clean_i_ready,                // bit 7: icache_miss
    x_valid && branch_mispredict,  // bit 6: branch_mispredict
    btb_take,                      // bit 5: btb_take
    x_valid && jump,               // bit 4: jump
    x_valid && is_branch,          // bit 3: is_branch
    fwd_active,                    // bit 2: fwd_active
    pipeline_stall,                // bit 1: pipeline_stall
    retire_valid                   // bit 0: retire_valid
  };

  // 【第四部分】高级控制逻辑单元 (BTB 动态分支目标缓冲预测器 + 机器模式 Trap 控制机)
  // 独立封装于 rtl/btb_trap_unit.v，满足竞赛规范中高级控制逻辑独立性要求
  wire [31:0] trap_in_cause = access_fault ? (is_store ? 32'd7 : 32'd5) : cause;
  wire [31:0] trap_in_tval  = access_fault ? address : tval;
  wire        csr_we        = complete && !take_trap && csr_access && csr_write;

  btb_trap_unit #(
    .RESET_MTVEC (RESET_MTVEC),
    .BTB_ENTRIES (32)
  ) u_btb_trap (
    .clk                   (clk),
    .reset                 (reset),

    // 1. BTB 取指预测与 EX 训练更新
    .fetch_pc              (fetch_pc),
    .btb_hit               (btb_hit),
    .btb_take              (btb_take),
    .btb_target            (btb_target),
    .btb_update_en         (complete && !take_trap && is_branch),
    .btb_update_pc         (x_pc),
    .btb_update_jump       (jump),
    .btb_update_target     (target),
    .btb_update_mispredict (branch_mispredict),
    .btb_hit_count         (btb_hit_count),
    .btb_miss_count        (btb_miss_count),

    // 2. 外部物理中断
    .timer_irq             (timer_irq),
    .ext_irq               (ext_irq),
    .soft_irq              (soft_irq),

    // 3. 流水线执行与中断握手
    .x_valid               (x_valid),
    .complete              (complete),
    .csr_access            (csr_access),
    .do_mret               (do_mret),
    .take_irq              (take_irq),
    .irq_target            (irq_target),

    // 4. 精确异常陷入
    .take_trap             (take_trap),
    .trap_in_pc            (x_pc),
    .trap_in_cause         (trap_in_cause),
    .trap_in_tval          (trap_in_tval),
    .irq_ret_pc            (irq_ret_pc),
    .mepc_out              (mepc_out),
    .mtvec_base            (mtvec_base),

    // 5. 特权 CSR 读写接口
    .csr_addr              (csr_addr),
    .csr_rdata             (csr_old),
    .csr_known             (csr_known),
    .csr_we                (csr_we),
    .csr_wdata             (csr_new),

    // 6. 性能计数器导入 (供 CSR 读取)
    .cycles                (cycles),
    .retired               (retired),

    // 7. 异常跟踪输出 (直接驱动顶层端口)
    .trap_valid            (trap_valid),
    .trap_pc               (trap_pc),
    .trap_cause            (trap_cause)
  );

  integer k; // 循环索引

  // 【第五部分】组合译码、执行与异常检测逻辑 (Combinational Decode & ALU)
  always @* begin
    // 赋默认值，彻底杜绝意外综合出 Latch (锁存器)
    legal        = 1'b1;
    regwrite     = 1'b0;
    is_load      = 1'b0;
    is_store     = 1'b0;
    is_branch    = 1'b0;
    jump         = 1'b0;
    do_mret      = 1'b0;
    result       = 32'b0;
    address      = 32'b0;
    target       = 32'b0;
    strobes      = 4'b0;
    store_data   = 32'b0;
    shifted_load = 32'b0;
    exception    = 1'b0;
    cause        = 32'b0;
    tval         = 32'b0;
    csr_access   = 1'b0;
    csr_write    = 1'b0;
    csr_new      = 32'b0;
    csr_source   = 32'b0;

    // 按操作码 (opcode) 主译码分支
    case (op)
      // LUI (Load Upper Immediate): rd = imm << 12
      7'h37: begin
        regwrite = 1'b1;
        result   = {x_insn[31:12], 12'b0};
      end

      // AUIPC (Add Upper Immediate to PC): rd = PC + (imm << 12)
      7'h17: begin
        regwrite = 1'b1;
        result   = x_pc + {x_insn[31:12], 12'b0};
      end

      // JAL (Jump and Link): rd = PC + 4, PC = PC + imm_j
      7'h6f: begin
        regwrite = 1'b1;
        result   = x_pc + 32'd4;
        jump     = 1'b1;
        target   = x_pc + imm_j;
      end

      // JALR (Jump and Link Register): rd = PC + 4, PC = (rs1 + imm_i) & ~1
      7'h67: begin
        legal    = (f3 == 3'b000);
        regwrite = 1'b1;
        result   = x_pc + 32'd4;
        jump     = 1'b1;
        target   = (a + imm_i) & 32'hfffffffe; // 最低位强制清零
      end

      // BRANCH: 条件分支指令 (BEQ, BNE, BLT, BGE, BLTU, BGEU)
      7'h63: begin
        is_branch = 1'b1;
        target    = x_pc + imm_b;
        case (f3)
          3'b000: jump = (a == b);                             // BEQ: 等于
          3'b001: jump = (a != b);                             // BNE: 不等于
          3'b100: jump = ($signed(a) <  $signed(b));           // BLT: 有符号小于
          3'b101: jump = ($signed(a) >= $signed(b));           // BGE: 有符号大于等于
          3'b110: jump = (a < b);                              // BLTU: 无符号小于
          3'b111: jump = (a >= b);                             // BGEU: 无符号大于等于
          default: legal = 1'b0;
        endcase
      end

      // OP-IMM: 寄存器与立即数整数运算 (ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI)
      7'h13: begin
        regwrite = 1'b1;
        case (f3)
          3'b000: result = a + imm_i;                          // ADDI (自然溢出截断)
          3'b010: result = {31'b0, ($signed(a) < $signed(imm_i))}; // SLTI: 有符号小于置位
          3'b011: result = {31'b0, (a < imm_i)};               // SLTIU: 无符号小于置位
          3'b100: result = a ^ imm_i;                          // XORI: 异或
          3'b110: result = a | imm_i;                          // ORI: 或
          3'b111: result = a & imm_i;                          // ANDI: 与
          3'b001: begin                                        // SLLI: 逻辑左移
            legal  = (f7 == 7'h00);
            result = a << x_insn[24:20];
          end
          3'b101: begin                                        // SRLI (f7=0) / SRAI (f7=0x20)
            legal = (f7 == 7'h00 || f7 == 7'h20);
            if (f7 == 7'h20)
              result = $signed(a) >>> x_insn[24:20];          // SRAI: 算术右移 (符号位填充)
            else
              result = a >> x_insn[24:20];                    // SRLI: 逻辑右移 (高位补零)
          end
        endcase
      end

      // OP: 寄存器与寄存器整数运算 (ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND)
      7'h33: begin
        regwrite = 1'b1;
        legal    = (f7 == 7'h00 || (f7 == 7'h20 && (f3 == 3'b000 || f3 == 3'b101)));
        case (f3)
          3'b000: result = (f7 == 7'h20) ? (a - b) : (a + b); // SUB (f7=0x20) / ADD (f7=0)
          3'b001: result = a << b[4:0];                       // SLL: 逻辑左移 (仅使用低 5 位)
          3'b010: result = {31'b0, ($signed(a) < $signed(b))}; // SLT: 有符号小于置位
          3'b011: result = {31'b0, (a < b)};                  // SLTU: 无符号小于置位
          3'b100: result = a ^ b;                             // XOR: 异或
          3'b101: begin                                       // SRL (f7=0) / SRA (f7=0x20)
            if (f7 == 7'h20)
              result = $signed(a) >>> b[4:0];                 // SRA: 算术右移
            else
              result = a >> b[4:0];                           // SRL: 逻辑右移
          end
          3'b110: result = a | b;                             // OR: 按位或
          3'b111: result = a & b;                             // AND: 按位与
        endcase
      end

      // LOAD: 访存加载指令 (LB, LH, LW, LBU, LHU)
      7'h03: begin
        is_load      = 1'b1;
        regwrite     = 1'b1;
        address      = a + imm_i;                             // 计算有效物理地址
        shifted_load = d_rdata >> (8 * address[1:0]);         // 针对未字对齐的偏移进行右移
        case (f3)
          3'b000: result = {{24{shifted_load[7]}},  shifted_load[7:0]};  // LB: 符号扩展字节
          3'b001: result = {{16{shifted_load[15]}}, shifted_load[15:0]}; // LH: 符号扩展半字
          3'b010: result = shifted_load;                                 // LW: 读整字 (32位)
          3'b100: result = {24'b0, shifted_load[7:0]};                   // LBU: 零扩展字节
          3'b101: result = {16'b0, shifted_load[15:0]};                  // LHU: 零扩展半字
          default: legal = 1'b0;
        endcase

        // 硬件对齐检查：LH/LHU 需 2 字节对齐 (addr[0]==0)；LW 需 4 字节对齐 (addr[1:0]==0)
        if (((f3 == 3'b001 || f3 == 3'b101) && address[0]) ||
            (f3 == 3'b010 && |address[1:0])) begin
          exception = 1'b1;
          cause     = 32'd4;                                  // mcause = 4: Load address misaligned
          tval      = address;                                // mtval 记录故障地址
        end
      end

      // STORE: 访存存储指令 (SB, SH, SW)
      7'h23: begin
        is_store   = 1'b1;
        address    = a + imm_s;                               // 计算存储物理地址
        store_data = b << (8 * address[1:0]);                 // 将待写数据移位对准对应字节通道
        case (f3)
          3'b000: strobes = 4'b0001 << address[1:0];         // SB: 单字节写使能
          3'b001: strobes = 4'b0011 << address[1:0];         // SH: 半字写使能
          3'b010: strobes = 4'b1111;                         // SW: 整字写使能
          default: legal  = 1'b0;
        endcase

        // 硬件对齐检查：SH 需 2 字节对齐；SW 需 4 字节对齐
        if ((f3 == 3'b001 && address[0]) || (f3 == 3'b010 && |address[1:0])) begin
          exception = 1'b1;
          cause     = 32'd6;                                  // mcause = 6: Store address misaligned
          tval      = address;                                // mtval 记录故障地址
        end
      end

      // FENCE: 内存屏障指令
      7'h0f: begin
        // 写穿缓存无待提交写队列；FENCE顺序完成，FENCE.I另行冲刷取指并失效I缓存
        legal = (f3 == 3'b000 || f3 == 3'b001);
      end

      // SYSTEM: 特权指令与 CSR 读写指令
      7'h73: begin
        if (f3 == 3'b000) begin
          // 系统控制指令 (ECALL, EBREAK, MRET)
          case (x_insn)
            32'h00000073: begin                               // ECALL: 环境调用
              exception = 1'b1;
              cause     = 32'd11;                             // mcause = 11: Environment call from M-mode
            end
            32'h00100073: begin                               // EBREAK: 断点
              exception = 1'b1;
              cause     = 32'd3;                              // mcause = 3: Breakpoint
              tval      = x_pc;
            end
            32'h30200073: begin                               // MRET: 从机器模式异常返回
              do_mret   = 1'b1;
            end
            default: legal = 1'b0;
          endcase
        end else begin
          // CSR 原子读写指令 (CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI)
          csr_access = 1'b1;
          regwrite   = 1'b1;
          result     = csr_old;                               // 原始 CSR 值写回 rd 通用寄存器
          // f3[2] 为 1 代表立即数类型 (zimm = rs1 编号零扩展)，否则为寄存器操作数
          csr_source = f3[2] ? {27'b0, rs1} : a;

          case (f3)
            3'b001, 3'b101: begin                             // CSRRW / CSRRWI: 覆盖写
              csr_write = 1'b1;
              csr_new   = csr_source;
            end
            3'b010, 3'b110: begin                             // CSRRS / CSRRSI: 按位置位
              csr_write = (rs1 != 5'd0);                      // rs1 为 0 时只读不写
              csr_new   = csr_old | csr_source;
            end
            3'b011, 3'b111: begin                             // CSRRC / CSRRCI: 按位清零
              csr_write = (rs1 != 5'd0);                      // rs1 为 0 时只读不写
              csr_new   = csr_old & ~csr_source;
            end
            default: legal = 1'b0;
          endcase

          // 非法检查：未实现的 CSR 或对只读 CSR (addr[11:10]==2'b11) 尝试写入，均触发非法指令异常
          if (!csr_known || (csr_write && csr_addr[11:10] == 2'b11)) begin
            legal = 1'b0;
          end
        end
      end

      default: legal = 1'b0;                                  // 未定义操作码，判定为非法指令
    endcase

    // 异常优先级裁决与覆盖 (按先后顺序：跳转目标未对齐 -> 非法指令 -> 取指错误)
    // 1. 跳转目标地址未对齐 (仅当分支成立或无条件跳转时检测目标地址低两位)
    if (jump && |target[1:0]) begin
      exception = 1'b1;
      cause     = 32'd0;                                      // mcause = 0: Instruction address misaligned
      tval      = target;
    end
    // 2. 指令编码非法 (覆盖前面的异常)
    if (!legal) begin
      exception = 1'b1;
      cause     = 32'd2;                                      // mcause = 2: Illegal instruction
      tval      = x_insn;                                     // mtval 保存非法指令机器码
    end
    // 3. 取指阶段已报告总线/对齐错误 (最高优先级覆盖)
    if (x_fetch_error) begin
      exception = 1'b1;
      cause     = 32'd1;                                      // mcause = 1: Instruction access fault
      tval      = x_pc;                                       // mtval 保存故障 PC
    end
  end

  // 【第六部分】时序状态更新与流水线推动 (Sequential Pipeline Registers)
  always @(posedge clk) begin
    if (reset) begin
      // 同步复位：清空流水线所有状态与寄存器
      fetch_pc      <= RESET_PC;
      f_valid       <= 1'b0;
      f_insn <= 32'h13; f_error <= 1'b0;
      f_pc          <= 32'b0;
      x_valid       <= 1'b0;
      x_pc          <= 32'b0;
      x_insn        <= 32'b0;
      x_fetch_error <= 1'b0;
      w_valid       <= 1'b0;
      w_we          <= 1'b0;
      w_rd          <= 5'b0;
      w_value       <= 32'b0;
      w_pc          <= 32'b0;
      w_insn        <= 32'b0;

      // 性能计数器清零
      cycles         <= 64'b0;
      retired        <= 64'b0;
      stall_cycles   <= 64'b0;
      branch_count   <= 64'b0;
      taken_count    <= 64'b0;

      // 分支预测流水线寄存器清零
      f_pred_taken   <= 1'b0;
      f_pred_target  <= 32'b0;
      x_pred_taken   <= 1'b0;
      x_pred_target  <= 32'b0;

      // 跟踪输出清零 (trap_trace 由 btb_trap_unit 驱动)
      retire_valid  <= 1'b0;
      retire_pc     <= 32'b0;
      retire_insn   <= 32'b0;
      retire_rd     <= 5'b0;
      retire_value  <= 32'b0;

      // 通用寄存器清零
      for (k = 0; k < 32; k = k + 1) begin
        regs[k] <= 32'b0;
      end
    end else begin
      // 周期计数器递增
      cycles <= cycles + 64'd1;

      // WB 级指令退休更新
      retire_valid <= w_valid;
      retire_pc    <= w_pc;
      retire_insn  <= w_insn;
      retire_rd    <= (w_we && w_rd != 5'd0) ? w_rd : 5'd0;
      retire_value <= (w_we && w_rd != 5'd0) ? w_value : 32'd0;

      // 指令正式提交：更新退休计数器，并将数据真正写入通用寄存器堆
      if (w_valid) begin
        retired <= retired + 64'd1;
        if (w_we && w_rd != 5'd0) begin
          regs[w_rd] <= w_value;
        end
      end
      // 硬件硬保障：x0 寄存器永远写不进任何非零值
      regs[0] <= 32'b0;

      // 异常与停顿统计
      if (x_valid && !complete) begin
        stall_cycles <= stall_cycles + 64'd1;
      end

      // 推进进入 WB 级的寄存器
      // 同步异常不退休；异步中断之前已完成的指令仍须退休，包括寄存器/访存副作用
      w_valid <= complete && !take_trap;
      w_we    <= regwrite;
      w_rd    <= rd;
      w_value <= result;
      w_pc    <= x_pc;
      w_insn  <= x_insn;

      // 条件分支统计 (Core 性能计数器)
      if (complete && !take_trap && is_branch) begin
        branch_count <= branch_count + 64'd1;
        if (jump) begin
          taken_count <= taken_count + 64'd1;
        end
      end

      // 流水线控制：重定向 (Redirect) vs 顺序推进 (Advance)
      if (redirect) begin
        // 发生重定向：计算下一跳目标 (异常入口 mtvec、中断入口 irq_target、MRET 返回点 mepc 或跳转分支 actual_target/target)
        fetch_pc <= redirect_target;
        // 冲刷流水线：清空 IF 级与 EX 级有效标记，丢弃错误路径上的后续指令
        f_valid  <= 1'b0;
        x_valid  <= 1'b0;
      end else begin
        // IF 缓冲与 EX 独立推进：缺指令时必须清 EX 有效位，不能重复退休。
        if (x_ready) begin
          x_valid <= f_valid;
          x_pc <= f_pc;
          x_insn <= f_insn;
          x_fetch_error <= f_error;
          x_pred_taken <= f_pred_taken;
          x_pred_target <= f_pred_target;
          f_valid <= 1'b0;
        end
        // ready 表示本次取指已经完成，地址在等待期间不变。
        if (i_en && clean_i_ready) begin
          f_valid <= 1'b1;
          f_pc <= fetch_pc;
          f_insn <= i_rdata;
          f_error <= i_error;
          f_pred_taken <= btb_take;
          f_pred_target <= btb_target;
          fetch_pc <= btb_take ? btb_target : fetch_pc + 32'd4;
        end
      end
      // 若处于等待 (如 x_ready == 0)，则流水线寄存器保持原值，自动产生硬件停顿 (Stall)
    end
  end

endmodule


