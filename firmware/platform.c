/*
 文件名称：platform.c
 功能与设计思路：
   本文件实现板级驱动支持包 (BSP)，包含软件 RX 缓冲环、微切片非阻塞延时与防撕裂原子计时。
   1. 软件 RX 环形缓冲区：开辟 128 字节软件 FIFO，防止单字节硬件串口在延时期间发生溢出丢包。
   2. 微切片非阻塞延时：在延时循环中以微切片穿插轮询串口接收，消除接收盲区。
   3. 64 位防撕裂原子计时：采用双读校验算法读取硬件高低 32 位定时器，确保计时单调准确。
*/

#include "platform.h"

/* 软件 RX 环形缓冲区配置 */
#define RX_FIFO_SIZE 128
static uint8_t rx_fifo[RX_FIFO_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;
static volatile uint8_t rx_sticky_error = 0;

/* 轮询硬件串口状态并读取接收数据至软件 FIFO */
static inline void uart_rx_poll(void) {
    uint32_t status = MMIO(UART_BASE + 4);
    if (status & 8u) {
        /* 读取并原子清除硬件端接收错误标志 */
        MMIO(UART_BASE + 12) = 1u;
        rx_sticky_error = 1;
    }
    if (status & 4u) {
        uint8_t byte = (uint8_t)(MMIO(UART_BASE + 8) & 0xffu);
        uint8_t next_head = (uint8_t)((rx_head + 1) & (RX_FIFO_SIZE - 1));
        if (next_head != rx_tail) {
            rx_fifo[rx_head] = byte;
            rx_head = next_head;
        } else {
            rx_sticky_error = 1;
        }
    }
}

/* 微切片非阻塞延时：每隔微小周期切片轮询一次 RX，保证等待期间不丢失串口命令 */
static inline void delay_cycles_cpu(uint32_t count) {
    while (count > 0) {
        uart_rx_poll();
        uint32_t step = (count > 100) ? 100 : count;
        count -= step;
        __asm__ volatile (
            "1: addi %0, %0, -1\n"
            "   bnez %0, 1b\n"
            : "+r" (step)
        );
    }
    uart_rx_poll();
}

/* 串口单字符发送 */
void putchar_uart(char c) {
    static uint32_t char_count = 0;
    while (!(MMIO(UART_BASE + 4) & 1u)) {
        uart_rx_poll();
    }
    MMIO(UART_BASE) = (uint8_t)c;
    while (!(MMIO(UART_BASE + 4) & 1u)) {
        uart_rx_poll();
    }

#ifndef SIM_TEST
    char_count++;
    if (c == '\n' || char_count >= 32) {
        char_count = 0;
        /* 换行或满 32 字节时微延时约 2ms，期间高频轮询 RX */
        delay_cycles_cpu(20000);
    } else {
        /* 单字符微延时，稳定链路 */
        delay_cycles_cpu(800);
    }
#else
    (void)char_count;
#endif
}

/* 字符串发送 */
void puts_uart(const char *s) {
    while (*s) {
        putchar_uart(*s++);
    }
}

/* 32 位十六进制打印 */
void print_hex(uint32_t v) {
    const char *h = "0123456789abcdef";
    for (int i = 7; i >= 0; i--) {
        putchar_uart(h[(v >> (i * 4)) & 15]);
    }
}

/* 32 位十进制打印 */
void print_dec(uint32_t v) {
    char b[11];
    unsigned n = 0;
    do {
        b[n++] = (char)('0' + v % 10);
        v /= 10;
    } while (v);
    while (n) {
        putchar_uart(b[--n]);
    }
}

/* 64 位硬件周期原子读取：通过三次采样校验高低位跨界溢出 */
uint64_t timer_cycles(void) {
    uint32_t h, l, h2;
    do {
        h  = MMIO(TIMER_BASE + 4);
        l  = MMIO(TIMER_BASE);
        h2 = MMIO(TIMER_BASE + 4);
    } while (h != h2);
    return ((uint64_t)h << 32) | l;
}

/* 获取硬件标定的处理器主频常数 */
uint32_t cpu_hz(void) {
    return MMIO(TIMER_BASE + 0x1c);
}

/* 非阻塞字符读取：从软件 FIFO 弹出单个字符，队列为空返回 -1 */
int getchar_uart(void) {
    uart_rx_poll();
    if (rx_head != rx_tail) {
        uint8_t ch = rx_fifo[rx_tail];
        rx_tail = (uint8_t)((rx_tail + 1) & (RX_FIFO_SIZE - 1));
        return (int)ch;
    }
    return -1;
}

/* 基础内存拷贝 */
void *memcpy(void *d, const void *s, size_t n) {
    unsigned char *a = d;
    const unsigned char *b = s;
    while (n--) {
        *a++ = *b++;
    }
    return d;
}

/* 基础内存填充 */
void *memset(void *d, int c, size_t n) {
    unsigned char *a = d;
    while (n--) {
        *a++ = (unsigned char)c;
    }
    return d;
}

/* 读取并清除串口硬件黏滞错误标志 */
int uart_rx_error_and_clear(void) {
    uart_rx_poll();
    if (rx_sticky_error) {
        rx_sticky_error = 0;
        return 1;
    }
    return 0;
}
