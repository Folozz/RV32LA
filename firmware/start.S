# 文件名称：start.S
# 功能与设计思路：
#   本文件为 RISC-V 32 位裸机启动引导程序与默认 Trap 异常向量入口。
#   1. 初始化全局指针寄存器 (gp) 与栈顶指针寄存器 (sp)。
#   2. 清零 .bss 未初始化全局数据段，建立标准 C 语言运行环境。
#   3. 配置机器模式异常向量基址寄存器 (mtvec) 为 Direct 模式。
#   4. 调用 C 语言主函数 main()；捕获返回值并写入系统监视端口。

.section .init, "ax", @progbits
.global _start

_start:
    # 1. 初始化全局指针 (gp)
    .option push
    .option norelax
    la      gp, __global_pointer$
    .option pop

    # 2. 初始化栈顶指针 (sp)，指向 64KB 片上 RAM 顶端
    la      sp, __stack_top

    # 3. 清零 BSS 数据段
    la      t0, __bss_start
    la      t1, __bss_end
1:
    bgeu    t0, t1, 2f
    sw      zero, 0(t0)
    addi    t0, t0, 4
    j       1b
2:

    # 4. 配置默认机器模式异常向量入口 (mtvec)
    la      t0, trap_entry
    csrw    mtvec, t0

    # 5. 调用 C 语言主程序
    call    main

    # 6. 正常退出：将 main 返回值写入仿真监视端口
    li      t0, 0x2000f000
    sw      a0, 0(t0)

3:
    # 主程序返回后挂起
    j       3b

.balign 4
trap_entry:
    # 默认异常服务入口：读取 mcause 并写入系统监视端口
    csrr    t1, mcause
    li      t0, 0x2000f000
    li      t2, 0xe0000000
    or      t1, t1, t2
    sw      t1, 0(t0)

4:
    j       4b
