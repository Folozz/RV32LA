/*
 文件名称：demo.c
 功能与设计思路：
   本文件为片上系统主控程序，包含上电自检、交互式命令调度与逻辑分析仪采样控制。
   1. 采样控制：支持配置分频、点数、触发条件与信号源，基于采样周期动态计算超时预算。
   2. 交互控制台：支持单字符快捷命令（i: 指令集自测, t: 中断自测, b: CoreMark 跑分, g: GPIO 读写, c: 采样捕获）。
*/

#include "platform.h"
#include "command.h"

#ifndef DISABLE_COREMARK
int coremark_main_renamed(void);
#endif

int run_isa_test(void);
int run_interrupt_test(void);

/* 逻辑分析仪采样捕获控制驱动 */
static int capture(uint32_t div, uint32_t len, uint32_t trig, uint32_t source) {
    /* 复位状态机 */
    MMIO(LA_BASE) = 2;

    /* 配置分频、长度、触发与信号源寄存器 */
    MMIO(LA_BASE + 8)  = div;
    MMIO(LA_BASE + 12) = len;
    MMIO(LA_BASE + 16) = trig;
    MMIO(LA_BASE + 24) = source;

    /* 启动采样 */
    MMIO(LA_BASE) = 1;

    uint64_t start = timer_cycles();

    /* 动态计算超时预算：触发等待预算 + 实际采样周期 (div * len) + 1 秒保护裕量 */
    uint64_t trig_budget   = (trig == 0) ? ((uint64_t)cpu_hz() * 2u) : ((uint64_t)cpu_hz() * 10u);
    uint64_t sample_cycles = (uint64_t)div * (uint64_t)len;
    uint64_t timeout       = trig_budget + sample_cycles + (uint64_t)cpu_hz();

    /* 等待采样完成或配置错误 */
    while (!(MMIO(LA_BASE + 4) & 6u)) {
        if (timer_cycles() - start > timeout) {
            MMIO(LA_BASE) = 2;
            puts_uart("ERR timeout\n");
            return 1;
        }
    }

    if (MMIO(LA_BASE + 4) & 4u) {
        puts_uart("ERR config\n");
        return 2;
    }

    /* 校验实际采集样本数 */
    uint32_t count = MMIO(LA_BASE + 20);
    uint32_t checksum = 0;
    if (count != len || count > CAPTURE_MAX_SAMPLES) {
        puts_uart("ERR count\n");
        return 3;
    }

    /* 输出采样头信息 */
    puts_uart("CAPTURE ");
    print_dec(div);
    putchar_uart(' ');
    print_dec(count);
    putchar_uart(' ');
    print_dec(cpu_hz());
    putchar_uart('\n');

    /* 读出双端口 RAM 中的样本并计算累加和 */
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t sample = MMIO(LA_BUFFER + 4 * i) & 255u;
        checksum += sample;
        print_hex(sample);
        putchar_uart('\n');
    }

    /* 输出校验结束标志 */
    puts_uart("END ");
    print_hex(checksum);
    putchar_uart('\n');
    return 0;
}

int main(void) {
    puts_uart("PANGO RV32I BENCHMARK AND DEMO\n");

    /* 硬件自检：验证 GPIO 控制器 */
    MMIO(GPIO_BASE) = 0x5a;
    if (MMIO(GPIO_BASE) != 0x5a) {
        puts_uart("ERR gpio\n");
        return 4;
    }

#ifdef SIM_TEST
    return capture(4, 64, 0, 0);
#else
    puts_uart("Commands: h; i; t; b; g; c 1 32 0 1\nREADY RV32LA\n");

    command_line line;
    command_line_init(&line);

    for (;;) {
        if (uart_rx_error_and_clear()) {
            command_line_rx_error(&line);
        }

        int ch = getchar_uart();
        if (ch < 0) {
            continue;
        }

        int event = command_line_feed(&line, (unsigned)ch);
        if (event == COMMAND_MORE) {
            continue;
        }
        if (event < 0) {
            puts_uart(event == COMMAND_RX_ERROR ? "ERR rx\n" : "ERR line\n");
            continue;
        }

        /* 处理单字符快捷指令 */
        if (line.text[0] != '\0' && line.text[1] == '\0') {
            if (line.text[0] == 'b') {
                puts_uart("BENCHMARK BEGIN; do not send capture commands until READY\n");
#ifndef DISABLE_COREMARK
                coremark_main_renamed();
#endif
                puts_uart("BENCHMARK RETURNED; inspect CRC and validity output\nREADY RV32LA\n");
                continue;
            } else if (line.text[0] == 'i') {
                puts_uart("ISA TEST BEGIN\n");
                int rc = run_isa_test();
                if (rc == 0) {
                    puts_uart("ISA TEST: PASS\nREADY RV32LA\n");
                } else {
                    puts_uart("ISA TEST: FAIL at check #");
                    print_dec((uint32_t)rc);
                    puts_uart("\nREADY RV32LA\n");
                }
                continue;
            } else if (line.text[0] == 't') {
                puts_uart("INTERRUPT TEST BEGIN\n");
                int rc = run_interrupt_test();
                if (rc == 0) {
                    puts_uart("INTERRUPT TEST: PASS\nREADY RV32LA\n");
                } else {
                    puts_uart("INTERRUPT TEST: FAIL code #");
                    print_dec((uint32_t)rc);
                    puts_uart("\nREADY RV32LA\n");
                }
                continue;
            } else if (line.text[0] == 'g') {
                puts_uart("GPIO IN ");
                print_hex(MMIO(GPIO_BASE + 4));
                puts_uart(" OUT ");
                print_hex(MMIO(GPIO_BASE));
                putchar_uart('\n');
                continue;
            } else if (line.text[0] == 'h') {
                puts_uart("Commands: h; i; t; b; g; c 1 32 0 1\nREADY RV32LA\n");
                continue;
            }
        }

        /* 处理采样指令 */
        capture_command command;
        if (!parse_capture_command(line.text, &command)) {
            puts_uart("ERR command\n");
            continue;
        }
        capture(command.divider, command.length, command.trigger, command.source);
    }
#endif
}
