# Pango-RV32LA

## 目录架构说明

| 目录 | 职责 |
|---|---|
| **docs** | 竞赛技术报告 、架构设计图等相关文档 | 
| **rtl** | 硬件电路源码与物理约束：8个Verilog模块与top.fdc，综合顶层为 fpga_top |
| **data** | 演示视频以及相关截图证明 |
| **firmware** | 裸机启动汇编、底层驱动、流式命令解析、逻辑分析仪应用及CoreMark平台移植桩 |
| **build** | 固件编译产物：唯一生成 demo.hex| 
| **scripts** | 自动化构建脚本 build.py（调用 GCC 编译并同步回写至 soc_ram.v） | 
| **tests** | RV32I 定向指令测试与中断异常自检 | 
| **third_party** | 官方 CoreMark 基准测试源码库| 

