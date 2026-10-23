/*
 链接脚本文件：link.ld
 功能与设计思路：
   本脚本规范 64KB 片上统一 SRAM 内存布局与段排布。
   1. 代码段 (.text) 与已初始化数据段 (.data) 紧凑排布，导出 __global_pointer$。
   2. 未初始化段 (.bss) 明确起始与结束边界供启动汇编清零。
   3. 栈顶设于 64KB 边界 (0x00010000)，静态断言保证预留至少 8KB 堆栈安全空间。
*/

OUTPUT_ARCH(riscv)
ENTRY(_start)

/* 定义物理存储器空间：64KB 单周期片上 RAM，基地址 0x00000000 */
MEMORY {
    RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 64K
}

SECTIONS {
    /* 1. 代码段与只读数据段 */
    .text : {
        KEEP(*(.init))
        *(.text*)
        *(.rodata*)
    } > RAM

    /* 2. 已初始化全局变量段 */
    .data : {
        . = ALIGN(4);
        __global_pointer$ = . + 0x800;
        *(.sdata*)
        *(.data*)
    } > RAM

    /* 3. 未初始化全局变量段 (BSS) */
    .bss (NOLOAD) : {
        . = ALIGN(4);
        __bss_start = .;
        *(.sbss*)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > RAM

    __image_end = .;

    /* 4. 堆栈顶指针定义 */
    __stack_top = ORIGIN(RAM) + LENGTH(RAM);

    /* 5. 编译期堆栈安全断言：静态预留至少 8KB 堆栈空间 */
    ASSERT(__image_end < __stack_top - 8192, "RAM overflow: preserve 8 KiB stack")

    /* 丢弃无用的调试注释与展开帧 */
    /DISCARD/ : {
        *(.comment)
        *(.eh_frame*)
    }
}
