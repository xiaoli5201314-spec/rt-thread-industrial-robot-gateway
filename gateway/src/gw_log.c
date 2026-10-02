/*
 * gw_log.c - 分级日志实现
 *
 * 输出格式：
 *   [  123456.789][E][BUS ] 文本
 *    ^单调毫秒   ^级别 ^标签
 * 固定宽度便于现场用串口工具肉眼比对时序。
 *
 * 大端/小端与 RTOS 无关；不做动态分配, 使用栈上 192 字节行缓冲,
 * 保证可以在中断上下文之外被任意线程调用而不产生堆碎片。
 */
#include "gw_log.h"
#include "port_rtos.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define GW_LOG_LINE_MAX 192u

static gw_log_level_t s_level = GW_LOG_INFO;

static const char s_level_char[] = { '-', 'E', 'W', 'I', 'D' };

void gw_log_set_level(gw_log_level_t level)
{
    if (level <= GW_LOG_DEBUG) {
        s_level = level;
    }
}

gw_log_level_t gw_log_get_level(void)
{
    return s_level;
}

bool gw_log_enabled(gw_log_level_t level)
{
    return (level != GW_LOG_NONE) && (level <= s_level);
}

void gw_log_printf(gw_log_level_t level, const char *tag, const char *fmt, ...)
{
    char line[GW_LOG_LINE_MAX];
    int  n;
    uint32_t ms;
    va_list ap;

    if (!gw_log_enabled(level)) {
        return;
    }

    ms = gw_port_tick_ms();

    /* 头部：时间 / 级别 / 标签。%.3u 打印毫秒内余数, 便于观察帧间隔 */
    n = snprintf(line, sizeof(line), "[%8lu.%03lu][%c][%-6s] ",
                 (unsigned long)(ms / 1000u),
                 (unsigned long)(ms % 1000u),
                 s_level_char[level],
                 (tag != NULL) ? tag : "-");
    if (n < 0) {
        return;
    }

    if ((size_t)n < sizeof(line)) {
        va_start(ap, fmt);
        {
            int m = vsnprintf(line + n, sizeof(line) - (size_t)n, fmt, ap);
            if (m > 0) {
                n += m;
            }
        }
        va_end(ap);
    }

    if ((size_t)n > sizeof(line) - 2u) {
        n = (int)(sizeof(line) - 2u);
    }
    line[n++] = '\n';
    line[n]   = '\0';

    gw_port_console_write(line, (size_t)n);
}
