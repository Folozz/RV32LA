/*
 文件名称：platform.h
 功能与设计思路：
   本文件定义片上外设基地址映射、寄存器访问宏及平台驱动抽象层接口。
   1. 外设地址划分：UART (0x20000000), GPIO (0x20001000), 定时器 (0x20002000), 逻辑分析仪 (0x20003000), 采样缓冲 (0x30000000)。
   2. 驱动接口声明：提供串口收发、格式化输出、64 位硬件周期计时及基础内存操作接口。
*/

#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stddef.h>

/* MMIO 物理寄存器访问宏 */
#define MMIO(a) (*(volatile uint32_t *)(uintptr_t)(a))

/* 外设物理基地址分配表 */
#define UART_BASE   0x20000000u  /* UART 串口控制器基地址 */
#define GPIO_BASE   0x20001000u  /* GPIO 控制器基地址 */
#define TIMER_BASE  0x20002000u  /* 64 位定时器与性能计数器基地址 */
#define LA_BASE     0x20003000u  /* 逻辑分析仪控制寄存器基地址 */
#define LA_BUFFER   0x30000000u  /* 逻辑分析仪采样数据缓冲基地址 */

/* 串口基础输出接口 */
void putchar_uart(char c);
void puts_uart(const char *s);

/* 格式化数字打印接口 */
void print_hex(uint32_t value);
void print_dec(uint32_t value);

/* 64 位硬件周期计时与主频获取接口 */
uint64_t timer_cycles(void);
uint32_t cpu_hz(void);

/* 串口非阻塞接收与错误检测接口 */
int getchar_uart(void);
int uart_rx_error_and_clear(void);

/* 基础内存操作接口 */
void *memcpy(void *dst, const void *src, size_t n);
void *memset(void *dst, int c, size_t n);

#endif /* PLATFORM_H */
