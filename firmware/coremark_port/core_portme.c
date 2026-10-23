/*
 文件名称：core_portme.c
 功能与设计思路：
   本文件实现 CoreMark 官方基准测试的底层计时与平台初始化桩函数。
   1. 高精度计时：start_time 与 stop_time 直接采样硬件 64 位周期计数器。
   2. 精度输出：在 portable_fini 中输出 64 位原始节拍 Hex 值，支持高精度性能折算。
*/

#include "coremark.h"
#include "platform.h"

#if VALIDATION_RUN
volatile ee_s32 seed1_volatile = 0x3415, seed2_volatile = 0x3415;
#else
volatile ee_s32 seed1_volatile = 0, seed2_volatile = 0;
#endif
volatile ee_s32 seed3_volatile = 0x66, seed4_volatile = ITERATIONS, seed5_volatile = 0;

ee_u32 default_num_contexts = 1;

static uint64_t start_tick, stop_tick;

/* 跑分计时启动 */
void start_time(void) {
    start_tick = timer_cycles();
}

/* 跑分计时停止 */
void stop_time(void) {
    stop_tick = timer_cycles();
}

/* 获取累计消耗的时钟周期数 */
CORE_TICKS get_time(void) {
    return stop_tick - start_tick;
}

/* 整数秒转换 */
secs_ret time_in_secs(CORE_TICKS ticks) {
    return (secs_ret)(ticks / cpu_hz());
}

/* 平台初始化 */
void portable_init(core_portable *p, int *argc, char *argv[]) {
    (void)argc;
    (void)argv;
    p->portable_id = 1;
    puts_uart("RV32I CoreMark port; use raw 64-bit ticks for precise results\n");
    puts_uart("CPU_HZ ");
    print_dec(cpu_hz());
    putchar_uart('\n');
}

/* 平台收尾：输出 64 位原始周期 Hex 值 */
void portable_fini(core_portable *p) {
    uint64_t dt = stop_tick - start_tick;
    puts_uart("RAW_TICKS_HEX ");
    print_hex((uint32_t)(dt >> 32));
    print_hex((uint32_t)dt);
    putchar_uart('\n');
    p->portable_id = 0;
}
