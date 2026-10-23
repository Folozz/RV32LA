# RV32LA 芯片级功能验证测试集

## 一、 文件及其职责说明

| 测试用例源文件 | 触发指令 | 覆盖测试要点与硬件验证方法 |
| :--- | :---: | :--- |
| **`isa.S`**  | **`i`** | **全部 48 条 RV32I/Zicsr/Traps 指令全覆盖**：<br>• 算术/逻辑：`ADD`, `SUB`, `SLL`, `SRL`, `SRA`, `SLT`, `SLTU`, `XOR`, `OR`, `AND` 及对应立即数指令；<br>• 分支跳转：`BEQ`, `BNE`, `BLT`, `BGE`, `BLTU`, `BGEU`, `JAL`, `JALR` 及 BTB 预测失误恢复；<br>• 访存操作：`LW`, `LH`, `LB`, `LHU`, `LBU`, `SW`, `SH`, `SB`；<br>• 冒险前递：RAW 数据前递与 Load-Use 冒险插入气泡；<br>• 内存同步屏障：`FENCE` 与 `FENCE.I`；<br>• 特权指令：`CSRRW`, `CSRRS`, `CSRRC`, `CSRRWI`, `CSRRSI`, `CSRRCI` 等 6 种 Zicsr 读写模式；<br>• 精确异常：指令非对齐、非法指令、访存未对齐等 10 大硬件 Trap 陷入与 MRET 恢复。 |
| **`interrupt_test.S`** | **`t`** | **机器模式（M-Mode）中断与向量表验证**：<br>• Direct 模式定时器中断（TIMECMP 比较匹配与清除）；<br>• Vectored 向量表模式软件中断（MSIP 触发、基址+4*cause 向量表精确跳转）；<br>• 硬件性能计数器读取（Cache 命中率、BTB 统计等）；<br>• 硬件中断使能栈压栈（`MPIE <= MIE; MIE <= 0`）与 `MRET` 原子恢复。 |

---

## 二、 交互测试执行方法（板上实时自检）

开发板烧录后，在串口助手（波特率 115200 8N1）中输入对应命令：

* **输入 `i` 回车**：立即执行全套指令与 10 大硬件异常测试，通过返回：
  ```text
  ISA TEST BEGIN
  ISA TEST: PASS
  READY RV32LA
  ```
* **输入 `t` 回车**：立即执行定时器中断、软件中断与向量表跳转测试，通过返回：
  ```text
  INTERRUPT TEST BEGIN
  INTERRUPT TEST: PASS
  READY RV32LA
  ```

