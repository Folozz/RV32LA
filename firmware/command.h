/*
 文件名称：command.h
 功能与设计思路：
   本文件定义串口流式命令行接收容器、错误码枚举及采样指令结构体。
   1. capture_command 结构体：包含分频、采样深度、触发模式与信号源配置。
   2. command_line_feed 接口：提供单字符流式驱动接口，支持非阻塞解析与错误黏滞处理。
*/

#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>

/* 单行命令行最大缓冲区容量：80 字节 */
#define COMMAND_LINE_CAPACITY 80u

/* 逻辑分析仪最大单次捕获深度：1024 样本点 */
#define CAPTURE_MAX_SAMPLES   1024u

/* 采样捕获控制参数结构体 */
typedef struct {
    uint32_t divider;
    uint32_t length;
    uint32_t trigger;
    uint32_t source;
} capture_command;

/* 命令行流式接收状态机容器 */
typedef struct {
    char     text[COMMAND_LINE_CAPACITY];
    uint32_t used;
    int      discard_error;
} command_line;

/* 状态机返回值枚举 */
enum {
    COMMAND_MORE     =  0,
    COMMAND_READY    =  1,
    COMMAND_TOO_LONG = -1,
    COMMAND_BAD_CHAR = -2,
    COMMAND_RX_ERROR = -3
};

/* 初始化命令行接收容器 */
void command_line_init(command_line *line);

/* 标记物理层接收错误 */
void command_line_rx_error(command_line *line);

/* 向状态机喂入单个字符并更新状态 */
int command_line_feed(command_line *line, unsigned byte);

/* 解析采样指令 (c <divider> <length> <trigger> <source>) */
int parse_capture_command(const char *text, capture_command *command);

#endif /* COMMAND_H */
