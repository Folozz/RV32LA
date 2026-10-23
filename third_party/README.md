# 第三方开源库说明 

本目录用于存放项目所依赖的权威第三方开源评测基准库。

---

## 1. EEMBC CoreMark 跑分测试库

### 1.1 开源仓库与下载来源
* **项目名称**：EEMBC CoreMark (v1.0)
* **官方主页**：[https://www.eembc.org/coremark/](https://www.eembc.org/coremark/)
* **官方 GitHub 仓库地址**：[https://github.com/eembc/coremark](https://github.com/eembc/coremark)

---

### 1.2 在本 SoC 工程中的集成与移植规范

1. **官方标准算法库（位于 `third_party/coremark/`，严格保持原版不变）**：
   * `core_main.c`：CoreMark 基准测试调度主逻辑；
   * `core_list_join.c`：单向链表遍历、插入与反转算法（测试数据相关与缓存局部性）；
   * `core_matrix.c`：矩阵乘法与求逆运算（测试流水线算术运算吞吐）；
   * `core_state.c`：状态机状态流转解析；
   * `core_util.c`：数据完整性与 CRC 校验函数；
   * `coremark.h`：官方核心数据结构与宏定义。

2. **自研 SoC 平台移植桩（位于 `firmware/coremark_port/`）**：
   * `core_portme.h`：针对自研 RV32I 纯整数核心配置编译选项（`HAS_FLOAT=0`、`MEM_METHOD=MEM_STATIC`、`MULTITHREAD=1`）；
   * `core_portme.c`：硬件计时桩，直接采样 SoC 底层 64 位无撕裂硬件运行周期计数器（`timer_cycles()`），输出高精度 `RAW_TICKS_HEX`；
   * `ee_printf.c`：专为 64 KiB 裸机内存环境定制的轻量级纯 C 格式化输出引擎，对接硬件串口驱动。

3. **自动化构建集成**：
   * 在执行 `python scripts/build.py` 构建主固件时，脚本自动将 `third_party/coremark/` 的算法核心源码与 `firmware/coremark_port/` 移植桩打包编译并内联至片上内存模型中。
