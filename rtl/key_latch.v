`timescale 1ns/1ps

// 模块名称：key_latch
// 功能与设计思路：
//   本模块实现 4 通道硬件按键消抖与边沿状态翻转锁存功能。
//   1. 双级触发器同步：消除机械按键输入端的亚稳态风险。
//   2. 数字积分消抖滤波：设定 5ms 时间窗口，只有在输入电平持续稳定后才更新滤波后输出。
//   3. 下降沿翻转锁存：检测到按键有效按下沿时翻转输出寄存器状态，将瞬态按键动作转换为稳定电平状态。

module key_latch #(
  parameter integer CLK_HZ      = 27000000,     // 系统主时钟频率，用于自动计算消抖计数阈值
  parameter integer DEBOUNCE_MS = 5             // 消抖窗口宽度，物理按键推荐 5~10ms
)(
  input  wire       clk,                         // 27MHz 系统同步时钟
  input  wire       reset,                       // 全局同步高有效复位信号
  input  wire [3:0] key_n,                       // 4 路板载低有效物理按键输入（按下为 0，松开为 1）
  output reg  [3:0] state_out                    // 4 路消抖锁存后的输出状态（点按翻转并持久保持）
);

  // 【阈值计算】：根据主频动态换算 5ms 所需的时钟周期数（27MHz 下约 135,000 个时钟周期）。
  // 此处使用了 CNT_MAX = (CNT_RAW < 1) ? 1 : CNT_RAW 的保护逻辑，
  // 保证在超低频仿真环境中测试时，模块仍可正常工作而不会发生除零或死锁。
  localparam integer CNT_RAW = (CLK_HZ / 1000) * DEBOUNCE_MS;
  localparam integer CNT_MAX = (CNT_RAW < 1) ? 1 : CNT_RAW;

  genvar i;
  generate
    for (i = 0; i < 4; i = i + 1) begin : g_key

      // 内部状态与寄存器定义
      // sync_q   : 两级同步寄存器链，消除异步按键输入的亚稳态
      // cnt      : 滤波积分计时器，记录当前电平已维持稳定的时钟周期数
      // stable   : 经消抖积分确认后的绝对稳定电平（1 = 松开，0 = 按下）
      // stable_d : stable 的打拍寄存器，用于在同步时钟域内检测“由高变低”的下降沿
      reg [1:0]  sync_q;
      reg [31:0] cnt;
      reg        stable;
      reg        stable_d;

      always @(posedge clk) begin
        // 第一步：两级触发器串联，消除跨时钟域亚稳态
        sync_q <= {sync_q[0], key_n[i]};

        if (reset) begin
          // 系统复位时，将按键默认初始化为松开状态 (1)，锁存状态初始化为熄灭 (0)
          cnt          <= 32'd0;
          stable       <= 1'b1;
          stable_d     <= 1'b1;
          state_out[i] <= 1'b0;
        end else begin
          // 第二步：连续稳定性判定
          // 当同步后的输入 sync_q[1] 与当前稳定电平 stable 不一致时，启动积分计时
          if (sync_q[1] != stable) begin
            if (cnt >= CNT_MAX - 1) begin
              // 唯有连续稳定持续满 5ms，才认可这是一次真正的电平切换
              stable <= sync_q[1];
              cnt    <= 32'd0;
            end else begin
              // 未满 5ms 继续累加
              cnt <= cnt + 32'd1;
            end
          end else begin
            // 若输入电平中途发生跳变（抖动回退），立即清零计数器重新计时，彻底滤除毛刺
            cnt <= 32'd0;
          end

          // 第三步：下降沿单脉冲提取与状态翻转
          // stable_d 为 1 且 stable 为 0 代表检测到用户“确实按下了按键”
          stable_d <= stable;
          if (stable_d && !stable) begin
            state_out[i] <= ~state_out[i];       // 状态翻转：按一下变高并保持，再按一下复原
          end
        end
      end

    end
  endgenerate

endmodule
