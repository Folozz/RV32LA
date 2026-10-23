# 文件名称：isa.S
# 功能与设计思路：
#   本文件实现 RISC-V 32 位基础整数指令集 (RV32I) 自动化回归自测套件。
#   1. 基础指令覆盖：覆盖 40 条基础整数运算、移位、分支、跳转、访存与立即数指令。
#   2. 微架构冒险：验证 WB->EX 数据前递网络与 BTB 动态分支预测循环学习。
#   3. 精确异常与 CSR：覆盖 10 类机器模式 Trap 异常触发、mcause/mtval 校验与 MRET 恢复。
#   4. 自测机制：53 项断言全部通过返回 0；失败返回故障检查点序号。

.section .text, "ax", @progbits
.global run_isa_test
.weak _start

_start:
    la      sp, __stack_top
    call    run_isa_test
    li      t0, 0x2000f000
    sw      a0, 0(t0)
1:
    j       1b

# 测试断言宏 expect
.macro expect reg, val, id
    li      a0, \id
    li      t6, \val
    bne     \reg, t6, isa_fail
.endm

run_isa_test:
    addi    sp, sp, -64
    sw      ra, 0(sp)
    sw      s0, 4(sp)
    sw      s1, 8(sp)
    sw      s2, 12(sp)
    sw      s3, 16(sp)
    sw      s4, 20(sp)
    sw      s5, 24(sp)
    sw      s6, 28(sp)
    sw      s7, 32(sp)
    sw      s8, 36(sp)
    sw      s9, 40(sp)
    sw      s10, 44(sp)
    sw      s11, 48(sp)
    csrr    t0, mtvec
    sw      t0, 52(sp)
    csrr    t0, mstatus
    sw      t0, 56(sp)

    la      t0, handler
    csrw    mtvec, t0
    li      s8, 0

    # 1. 验证 zero 寄存器只读为 0
    li      zero, 77
    expect  zero, 0, 1

    # 2-7. 移位类指令 (SRAI, SRLI, SRA, SRL, SLL, SLLI)
    li      t0, 0x80000001
    srai    t1, t0, 1
    expect  t1, 0xc0000000, 2
    srli    t1, t0, 1
    expect  t1, 0x40000000, 3
    li      t2, 31
    sra     t1, t0, t2
    expect  t1, -1, 4
    srl     t1, t0, t2
    expect  t1, 1, 5
    sll     t1, t0, t2
    expect  t1, 0x80000000, 6
    slli    t1, t0, 4
    expect  t1, 16, 7

    # 8-19. 算术与逻辑类指令 (ADD, SUB, SLT, SLTU, SLTI, SLTIU, XOR, XORI, AND, ANDI, OR, ORI)
    li      t0, -7
    li      t1, 12
    add     t2, t0, t1
    expect  t2, 5, 8
    sub     t2, t0, t1
    expect  t2, -19, 9
    slt     t2, t0, t1
    expect  t2, 1, 10
    sltu    t2, t0, t1
    expect  t2, 0, 11
    slti    t2, t0, -6
    expect  t2, 1, 12
    sltiu   t2, t0, -6
    expect  t2, 1, 13
    xor     t2, t0, t1
    expect  t2, -11, 14
    xori    t2, t0, 15
    expect  t2, -10, 15
    and     t2, t0, t1
    expect  t2, 8, 16
    andi    t2, t0, 15
    expect  t2, 9, 17
    or      t2, t0, t1
    expect  t2, -3, 18
    ori     t2, t0, 2
    expect  t2, -5, 19

    # 20-21. U 型立即数指令 (LUI, AUIPC)
    lui     t0, 0xabcde
    expect  t0, 0xabcde000, 20
1:  auipc   t0, 0
    la      t1, 1b
    li      a0, 21
    bne     t0, t1, isa_fail

    # 22-29. 访存与符号扩展指令 (SW, LW, LB, LBU, LH, LHU, SB, SH) 及 RAW 数据前递
    li      s0, 0xc000
    li      t0, 0x80ff7f01
    sw      t0, 0(s0)
    lw      t1, 0(s0)
    expect  t1, 0x80ff7f01, 22
    lb      t1, 3(s0)
    expect  t1, -128, 23
    lbu     t1, 3(s0)
    expect  t1, 128, 24
    lh      t1, 2(s0)
    expect  t1, -32513, 25
    lhu     t1, 2(s0)
    expect  t1, 33023, 26
    li      t0, 0x55
    sb      t0, 1(s0)
    li      t0, 0x1234
    sh      t0, 2(s0)
    lw      t1, 0(s0)
    expect  t1, 0x12345501, 27
    lw      t1, 0(s0)
    addi    t2, t1, 1
    expect  t2, 0x12345502, 28
    lw      t1, 0(s0)
    sw      t1, 4(s0)
    lw      t2, 4(s0)
    expect  t2, 0x12345501, 29

    # 30. 内存屏障指令 (FENCE 与 FENCE.I)
    fence   iorw, iorw
    .word   0x0000100f

    # 31. 条件分支指令 (BEQ, BNE, BGE, BLT, BLTU, BGEU)
    li      a0, 30
    li      t0, -1
    li      t1, 1
    beq     t0, t1, isa_fail
    bne     t0, t0, isa_fail
    bge     t0, t1, isa_fail
    blt     t1, t0, isa_fail
    bltu    t0, t1, isa_fail
    bgeu    t1, t0, isa_fail
    beq     t0, t0, 2f
    sw      zero, 0(s0)
2:  bne     t0, t1, 3f
    j       isa_fail
3:  blt     t0, t1, 4f
    j       isa_fail
4:  bge     t1, t0, 5f
    j       isa_fail
5:  bltu    t1, t0, 6f
    j       isa_fail
6:  bgeu    t0, t1, 7f
    j       isa_fail
7:  lw      t1, 0(s0)
    expect  t1, 0x12345501, 31

    # 32. 循环分支与 BTB 分支预测学习
    li      t0, 8
8:  addi    t0, t0, -1
    bne     t0, zero, 8b
    expect  t0, 0, 32

    # 33-34. 无条件跳转指令 (JAL, JALR)
    jal     ra, callee
    expect  t3, 66, 33
    la      t0, jalr_target
    ori     t0, t0, 1
    jalr    ra, 0(t0)
    expect  t3, 77, 34

    # 35-41. 特权 CSR 指令 (CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI, CSRR)
    li      t0, 0x123
    csrrw   t1, mscratch, t0
    expect  t1, 0, 35
    li      t0, 0x400
    csrrs   t1, mscratch, t0
    expect  t1, 0x123, 36
    csrrc   t1, mscratch, t0
    expect  t1, 0x523, 37
    csrrwi  t1, mscratch, 7
    expect  t1, 0x123, 38
    csrrsi  t1, mscratch, 8
    expect  t1, 7, 39
    csrrci  t1, mscratch, 3
    expect  t1, 15, 40
    csrr    t1, mscratch
    expect  t1, 12, 41

    # 42-52. 硬件精确同步异常与 MRET 返回测试
    # (1) ECALL 环境调用 (mcause = 11)
    li      a0, 42
    la      s9, after_ecall
    li      s10, 11
    li      s11, 0
    ecall
    j       isa_fail
after_ecall:

    # (2) EBREAK 断点异常 (mcause = 3)
    li      a0, 43
    la      s9, after_ebreak
    li      s10, 3
    la      s11, break_site
break_site:
    ebreak
    j       isa_fail
after_ebreak:

    # (3) 非法指令异常 (mcause = 2)
    li      a0, 44
    la      s9, after_illegal
    li      s10, 2
    li      s11, -1
    .word   0xffffffff
    j       isa_fail
after_illegal:

    # (4) Load 地址未对齐异常 (mcause = 4)
    li      a0, 45
    la      s9, after_lmis
    li      s10, 4
    li      s11, 0xc001
    lw      t0, 1(s0)
    j       isa_fail
after_lmis:

    # (5) Store 地址未对齐异常 (mcause = 6)
    li      a0, 46
    la      s9, after_smis
    li      s10, 6
    li      s11, 0xc001
    sh      zero, 1(s0)
    j       isa_fail
after_smis:

    # (6) 跳转地址未对齐异常 (mcause = 0)
    li      a0, 47
    la      s9, after_jmis
    li      s10, 0
    li      s11, 2
    jalr    t0, 2(zero)
    j       isa_fail
after_jmis:

    # (7) Load 总线访问错误 (mcause = 5)
    li      a0, 48
    la      s9, after_lfault
    li      s10, 5
    li      s11, 0x40000000
    lw      t0, 0(s11)
    j       isa_fail
after_lfault:

    # (8) Store 总线访问错误 (mcause = 7)
    li      a0, 49
    la      s9, after_sfault
    li      s10, 7
    li      s11, 0x40000000
    sw      zero, 0(s11)
    j       isa_fail
after_sfault:

    # (9) 取指越界访问错误 (mcause = 1)
    li      a0, 50
    la      s9, after_ifault
    li      s10, 1
    li      s11, 0x10000
    jalr    zero, 0(s11)
    j       isa_fail
after_ifault:

    # (10) 只读 CSR 写入触发非法指令 (mcause = 2)
    li      a0, 51
    la      s9, after_csr_ro
    li      s10, 2
    la      t0, ro_site
    lw      s11, 0(t0)
ro_site:
    csrrw   zero, cycle, zero
    j       isa_fail
after_csr_ro:

    # 验证 10 类异常全部成功捕获并恢复
    expect  s8, 10, 52

    # 全部测试通过，返回 0
    li      a0, 0
    j       isa_exit

callee:
    li      t3, 66
    ret

jalr_target:
    li      t3, 77
    ret

.balign 4
handler:
    csrr    t0, mcause
    bne     t0, s10, isa_fail
    csrr    t0, mtval
    bne     t0, s11, isa_fail
    csrw    mepc, s9
    addi    s8, s8, 1
    mret

isa_fail:
    # 失败出口：a0 保存当前失败测试点 ID

isa_exit:
    lw      t0, 56(sp)
    csrw    mstatus, t0
    lw      t0, 52(sp)
    csrw    mtvec, t0
    lw      s11, 48(sp)
    lw      s10, 44(sp)
    lw      s9, 40(sp)
    lw      s8, 36(sp)
    lw      s7, 32(sp)
    lw      s6, 28(sp)
    lw      s5, 24(sp)
    lw      s4, 20(sp)
    lw      s3, 16(sp)
    lw      s2, 12(sp)
    lw      s1, 8(sp)
    lw      s0, 4(sp)
    lw      ra, 0(sp)
    addi    sp, sp, 64
    ret
