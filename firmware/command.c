/*
 文件名称：command.c
 功能与设计思路：
   本文件实现流式命令行状态机与采样指令语法语义解析。
   1. 流式状态机：兼容 CRLF 终端换行，支持超长行丢弃与非法控制字符过滤。
   2. 错误黏滞机制：单行出错时静默丢弃后续字符直至换行，防止畸变指令拼合。
   3. 无除法防溢出：采用常数边界比对实现 32 位整型防溢出解析，避免引入软除法开销。
*/

#include "command.h"

/* 空白分隔符判断 */
static int separator(char c) {
    return c == ' ' || c == '\t';
}

/* 初始化命令行容器 */
void command_line_init(command_line *line) {
    line->used = 0;
    line->discard_error = 0;
    line->text[0] = '\0';
}

/* 标记接收错误并进入丢弃模式 */
void command_line_rx_error(command_line *line) {
    line->used = 0;
    line->discard_error = COMMAND_RX_ERROR;
}

/* 流式字符喂入状态机 */
int command_line_feed(command_line *line, unsigned byte) {
    if (byte == '\r') {
        return COMMAND_MORE; /* 静默忽略 \r，兼容 CRLF 换行 */
    }
    if (byte == '\n') {
        int result = line->discard_error ? line->discard_error : COMMAND_READY;
        if (line->discard_error) {
            line->text[0] = '\0';
        } else {
            line->text[line->used] = '\0';
        }
        line->used = 0;
        line->discard_error = 0;
        return result;
    }
    if (line->discard_error) {
        return COMMAND_MORE; /* 处于错误状态，丢弃后续字符直至换行 */
    }
    if ((byte < 32 && byte != '\t') || byte > 126) {
        line->discard_error = COMMAND_BAD_CHAR;
        return COMMAND_MORE;
    }
    if (line->used >= COMMAND_LINE_CAPACITY - 1) {
        line->discard_error = COMMAND_TOO_LONG;
        return COMMAND_MORE;
    }
    line->text[line->used++] = (char)byte;
    return COMMAND_MORE;
}

/* 解析采样指令：c <divider> <length> <trigger> <source> */
int parse_capture_command(const char *text, capture_command *command) {
    uint32_t values[4] = {0, 0, 0, 0};
    const char *p = text;

    if (*p++ != 'c' || !separator(*p)) {
        return 0;
    }

    for (unsigned field = 0; field < 4; ++field) {
        while (separator(*p)) {
            ++p;
        }
        if (*p < '0' || *p > '9') {
            return 0;
        }
        while (*p >= '0' && *p <= '9') {
            uint32_t digit = (uint32_t)(*p - '0');
            /* 常量边界比对防溢出 (429496729 * 10 + 5 == UINT32_MAX) */
            if (values[field] > 429496729u ||
                (values[field] == 429496729u && digit > 5u)) {
                return 0;
            }
            values[field] = values[field] * 10u + digit;
            ++p;
        }
        if (field < 3 && !separator(*p)) {
            return 0;
        }
    }

    while (separator(*p)) {
        ++p;
    }
    if (*p != '\0') {
        return 0;
    }

    /* 参数范围与语义合法性校验 */
    if (values[0] == 0 || values[1] == 0 || values[1] > CAPTURE_MAX_SAMPLES) {
        return 0;
    }
    if (values[3] > 1) {
        return 0;
    }

    /* 触发模式校验：仅允许立即模式 (0) 或通道边沿模式 (最低位为 1，通道号限定 0~7) */
    if ((values[2] & ~0x701u) != 0) {
        return 0;
    }
    if ((values[2] & 1u) == 0 && values[2] != 0) {
        return 0;
    }

    command->divider = values[0];
    command->length  = values[1];
    command->trigger = values[2];
    command->source  = values[3];
    return 1;
}
