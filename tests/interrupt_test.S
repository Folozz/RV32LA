# 文件名称：interrupt_test.S
# 功能与设计思路：
#   本文件实现 RISC-V 机器模式特权中断控制器自动化验证套件。
#   1. 特权 CSR 读写：验证 mie、mstatus、mtvec 寄存器配置与掩码。
#   2. Direct 模式中断：验证定时器中断 (MTI) 触发、服务程序跳转与现场恢复。
#   3. Vectored 模式中断：验证软件中断 (MSI) 经向量表基址加偏移查表跳转。
#   4. 性能计数器可达性：读取 Cache 与 BTB 硬件计数器 MMIO 寄存器。

.section .text, "ax", @progbits
.global run_interrupt_test
.weak _start

_start:
    la      sp, __stack_top
    call    run_interrupt_test
    li      t0, 0x2000f000
    sw      a0, 0(t0)
1:
    j       1b

run_interrupt_test:
    addi    sp, sp, -32
    sw      ra, 0(sp)
    sw      s1, 4(sp)
    sw      s2, 8(sp)
    sw      s3, 12(sp)
    sw      s4, 16(sp)
    csrr    t0, mtvec
    sw      t0, 20(sp)
    csrr    t0, mstatus
    sw      t0, 24(sp)
    csrr    t0, mie
    sw      t0, 28(sp)

    # 步骤 1: 特权 CSR 读写测试
    li      t1, 0x888
    csrw    mie, t1
    csrr    t2, mie
    bne     t1, t2, fail_csr

    la      t0, direct_handler
    csrw    mtvec, t0
    csrr    t1, mtvec
    bne     t0, t1, fail_csr

    # 步骤 2: Direct 模式定时器中断测试
    li      s1, 0
    li      s2, 0
    csrr    t0, 0xc00
    addi    t0, t0, 60
    li      t1, 0x20002020
    sw      t0, 0(t1)
    li      t1, 0x20002024
    sw      zero, 0(t1)

    li      t0, 0x8
    csrs    mstatus, t0

wait_timer:
    addi    s2, s2, 1
    li      t0, 200000
    bge     s2, t0, fail_timer
    beqz    s1, wait_timer

    li      t0, 0x8
    csrc    mstatus, t0
    li      t1, 1
    bne     s1, t1, fail_timer

    # 步骤 3: Vectored 模式软件中断测试
    li      s3, 0
    li      s4, 0
    la      t0, vector_table
    ori     t0, t0, 1           # MODE = 1 (Vectored)
    csrw    mtvec, t0

    li      t0, 0x8
    csrs    mstatus, t0

    li      t1, 0x20002028
    li      t2, 1
    sw      t2, 0(t1)

wait_soft:
    addi    s4, s4, 1
    li      t0, 200000
    bge     s4, t0, fail_soft
    beqz    s3, wait_soft

    li      t0, 0x8
    csrc    mstatus, t0
    li      t1, 1
    bne     s3, t1, fail_soft

    # 步骤 4: Cache 与 BTB 性能计数器 MMIO 读取校验
    li      t0, 0x20002030
    lw      t1, 0(t0)
    li      t0, 0x20002040
    lw      t2, 0(t0)
    li      t0, 0x20002050
    lw      t3, 0(t0)

pass_irq:
    li      a0, 0
    j       irq_exit

fail_csr:
    li      a0, 1
    j       irq_exit

fail_timer:
    li      a0, 2
    j       irq_exit

fail_soft:
    li      a0, 3
    j       irq_exit

fail_vec:
    li      a0, 4
    j       irq_exit

irq_exit:
    lw      t0, 28(sp)
    csrw    mie, t0
    lw      t0, 24(sp)
    csrw    mstatus, t0
    lw      t0, 20(sp)
    csrw    mtvec, t0
    lw      s4, 16(sp)
    lw      s3, 12(sp)
    lw      s2, 8(sp)
    lw      s1, 4(sp)
    lw      ra, 0(sp)
    addi    sp, sp, 32
    ret

.balign 4
direct_handler:
    csrr    t0, mcause
    li      t1, 0x80000007
    bne     t0, t1, fail_timer
    li      t0, 0x20002020
    sw      zero, 0(t0)
    li      t0, 0x20002024
    sw      zero, 0(t0)
    li      s1, 1
    mret

.balign 64
vector_table:
    j       bad_trap            # 0: User Software Interrupt
    j       bad_trap            # 1: Supervisor Software Interrupt
    j       bad_trap            # 2: Reserved
    j       m_soft_handler      # 3: Machine Software Interrupt (offset +12)
    j       bad_trap            # 4: User Timer Interrupt
    j       bad_trap            # 5: Supervisor Timer Interrupt
    j       bad_trap            # 6: Reserved
    j       m_timer_handler     # 7: Machine Timer Interrupt (offset +28)
    j       bad_trap            # 8: User External Interrupt
    j       bad_trap            # 9: Supervisor External Interrupt
    j       bad_trap            # 10: Reserved
    j       m_ext_handler       # 11: Machine External Interrupt (offset +44)

bad_trap:
    j       fail_vec

m_soft_handler:
    csrr    t0, mcause
    li      t1, 0x80000003
    bne     t0, t1, fail_soft
    li      t0, 0x20002028
    sw      zero, 0(t0)
    li      s3, 1
    mret

m_timer_handler:
    j       direct_handler

m_ext_handler:
    mret
