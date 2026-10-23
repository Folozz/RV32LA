`timescale 1ns/1ps

// 模块名称：logic_analyzer
// 功能与设计思路：
//   设计了这个完全集成于片上的 8 通道高速数字逻辑分析仪（ILA）硬件加速核。
//
//   1. 【硬件资源与存储架构设计】：
//      - 缓冲区设计：内置 1024 深度 × 8 位专用的片上双口 RAM（buffer_mem），完美推断为专用块 RAM；
//      - 双口无冲突分离：端口 A 由硬件采样状态机独立进行高速单拍写入；端口 B 作为独立同步只读口，
//        直接映射到 CPU MMIO 物理地址空间（0x30000000 ~ 0x30000FFF），采样完成后由 CPU 流水线按需读取，
//        彻底解耦了硬件采集与软件通信两大时钟活动。
//
//   2. 【宽范围时钟分频器 (1 ~ 65535)】：
//      - 支持 1~65535 级动态时钟分频。当 div=1 时，采样器以 27.000 MHz 系统主频全速无损抓取
//        CPU 流水线每拍的微架构信号（指令提交、停顿气泡、前递激活等）；当用于低频外设观测时，
//        可配置更大分频，观测时间窗口可拓宽至数秒。
//
//   3. 【智能触发控制引擎】：
//      - 立即触发模式 (Immediate Trigger, mode=0)：启动即开始采样；
//      - 通道上升沿触发模式 (Edge Trigger, mode=1)：通过 `cfg_trigger[10:8]` 动态指定 0~7 号目标通道，
//        硬件内部每拍登记历史电平 previous，利用组合逻辑 `!previous[ch] && samples[ch]` 精准识别
//        跳变沿，捕捉关键断点或异常现场。
//
//   4. 【控制优先级与状态机鲁棒性】：
//      - 控制优先级强制规定为：强制取消 (abort) > 启动 (start) > 连续采样 (sampling)；
//      - 启动时自动完成边界安全检查（拒绝零分频、零长度、超深度 1024 或非法触发模式），
//        避免非法配置导致硬件死锁。

module logic_analyzer #(
  parameter integer DEPTH = 1024                // 最大硬件采样样本深度（8 位 × 1024 点）
)(
  input  wire        clk,                       // 27MHz 系统同步采样主时钟
  input  wire        reset,                     // 全局同步高有效复位信号
  input  wire        start,                     // 采样启动脉冲（写 MMIO 控制寄存器 bit 0）
  input  wire        abort_capture,             // 采样强制取消脉冲（写 MMIO 控制寄存器 bit 1）

  // 配置寄存器输入
  input  wire [31:0] cfg_div,                   // 采样间隔分频系数（时钟周期数，1 ~ 65535）
  input  wire [31:0] cfg_length,                // 期望采样的样本总点数（1 ~ 1024）
  input  wire [31:0] cfg_trigger,               // 触发模式配置：[1:0]=0立即/1上升沿；[10:8]=通道号(0~7)
  input  wire [7:0]  samples,                   // 8 通道被测数字信号输入总线（已在顶层同步）

  // 状态指示输出（映射至 MMIO 状态寄存器）
  output reg         busy,                      // 1 = 采样器正在等待触发或正在写入数据
  output reg         done,                      // 1 = 本次采样已完全结束，缓冲数据就绪
  output reg         error,                     // 1 = 参数越界或在 busy 期间发生重复启动冲突
  output reg         triggered,                 // 1 = 触发条件已满足（立即触发时启动同拍置位）
  output reg  [31:0] count,                     // 已经成功写入缓冲区的实际样本点数

  // 专用同步读口（连接 SoC 数据总线 MMIO 窗口）
  input  wire        rd_en,                     // CPU 总线读使能（MMIO 地址命中 0x30000000 时有效）
  input  wire [31:0] rd_index,                  // 样本读取索引（地址偏移 / 4）
  output reg  [31:0] rd_data                    // 读出的 32 位数据（低 8 位为样本值，高 24 位自动补 0）
);

  // 【存储介质】：1024 点 8 位专用高速采样双口 RAM。
  // 注意：此处不设计复位清零循环，既节省了大量的门级逻辑资源，又完全符合 DRM 综合推断模板。
  reg [7:0]  buffer_mem [0:DEPTH-1];

  // 内部工作寄存器
  reg [31:0] div_latched;                       // 启动时锁存的采样分频系数
  reg [31:0] len_latched;                       // 启动时锁存的采样总点数
  reg [31:0] trig_latched;                      // 启动时锁存的触发模式与目标通道配置
  reg [31:0] tick_count;                        // 分频倒计时计数器（0 时写一个样本并重置为 div_latched-1）
  reg [7:0]  previous;                          // 上一拍的信号历史采样值，用于精准比对边沿跳变

  // 【边沿检测组合逻辑】：
  // 仅在 trig_latched[10:8] 指定的目标通道上，当上一拍为 0 且本拍为 1 时，确认上升沿命中。
  // 设计为每个系统时钟周期都实时检查一次，保证不会因为分频采样而漏掉短脉冲边沿。
  wire edge_match = !previous[trig_latched[10:8]] && samples[trig_latched[10:8]];

  // 【时序逻辑：双口 RAM 读操作与状态机调度】
  always @(posedge clk) begin
    // 独立同步读口：时钟上升沿根据 CPU 提供的索引读出样本数据，越界访问安全返回 0
    if (rd_en)
      rd_data <= (rd_index < DEPTH) ? {24'b0, buffer_mem[rd_index]} : 32'b0;

    if (reset) begin
      // 系统复位：重置所有控制状态位，保持采样器处于空闲与安全状态
      busy         <= 1'b0;
      done         <= 1'b0;
      error        <= 1'b0;
      triggered    <= 1'b0;
      count        <= 32'd0;
      div_latched  <= 32'd1;
      len_latched  <= 32'd0;
      trig_latched <= 32'd0;
      tick_count   <= 32'd0;
      previous     <= 8'd0;
      rd_data      <= 32'd0;
    end else begin
      // 每拍保存当前信号输入，为下一拍边沿比较提供基准
      previous <= samples;

      // 【第一优先级：强制取消采样 (Abort)】
      // 无论系统当前是处于等待触发还是正在采样，只要收到 abort 脉冲，立即强制停机。
      if (abort_capture) begin
        busy      <= 1'b0;
        done      <= 1'b0;
        triggered <= 1'b0;
      end

      // 【第二优先级：下发启动命令 (Start)】
      else if (start) begin
        // 若当前采样器正忙（尚未完成前次任务），再次启动视为非法冲突，置位 error 拒绝改写
        if (busy) begin
          error <= 1'b1;
        end else begin
          done       <= 1'b0;
          count      <= 32'd0;
          tick_count <= 32'd0;
          triggered  <= 1'b0;

          // 严格的安全边界检查：
          // 拒绝点数为 0、点数超过最大深度 1024、分频比为 0 或触发配置保留位违规的参数
          if (cfg_length == 0 || cfg_length > DEPTH || cfg_div == 0 ||
              (cfg_trigger & ~32'h00000701) != 0 ||
              (!cfg_trigger[0] && cfg_trigger != 0)) begin
            error <= 1'b1;
            busy  <= 1'b0;
          end else begin
            // 参数合法：锁定本次运行参数，并正式转入 busy 工作态
            error        <= 1'b0;
            busy         <= 1'b1;
            div_latched  <= cfg_div;
            len_latched  <= cfg_length;
            trig_latched <= cfg_trigger;
            // 立即触发模式下，triggered 标志同拍置 1，免除等待；边沿触发则初始为 0
            triggered    <= (cfg_trigger[1:0] == 0);
          end
        end
      end

      // 【第三优先级：运行状态机 (Waiting Trigger -> Sampling Data)】
      else if (busy) begin
        if (!triggered) begin
          // 阶段 A：等待目标通道出现有效跳变沿
          if (edge_match) begin
            triggered  <= 1'b1;                  // 触发命中！
            tick_count <= 32'd0;                 // 清零倒计时，准备在首个采样时刻写入第 1 个样本
          end
        end else if (tick_count == 0) begin
          // 阶段 B：分频倒计时归零，执行一次硬件写入
          buffer_mem[count] <= samples;          // 将当前被测 8 路总线锁存入双口 RAM
          count             <= count + 32'd1;    // 采样点数指针递增
          tick_count        <= div_latched - 1;  // 重装分频倒计时计数器

          // 阶段 C：终点判定。达到用户设定的点数时，完美结束本次采集任务
          if (count + 1 >= len_latched) begin
            busy <= 1'b0;
            done <= 1'b1;                        // 宣布采集完成，CPU 可随时安全读回数据
          end
        end else begin
          tick_count <= tick_count - 32'd1;      // 分频倒计时递减
        end
      end
    end
  end

endmodule
