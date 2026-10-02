/*
 * gw_log.h - 分级日志
 *
 * 目标板上走 rt_kprintf，PC 仿真下走 stderr；输出通道由 port 层提供，
 * 本模块不直接包含 stdio.h，保证目标板编译时无宿主依赖。
 */
#ifndef GW_LOG_H
#define GW_LOG_H

#include "gw_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GW_LOG_NONE  = 0,   /* 关闭全部日志                                */
    GW_LOG_ERROR = 1,   /* 必须上报的故障                              */
    GW_LOG_WARN  = 2,   /* 异常但可恢复                                */
    GW_LOG_INFO  = 3,   /* 关键状态迁移（默认级别）                    */
    GW_LOG_DEBUG = 4    /* 逐帧细节, 现场调试用                        */
} gw_log_level_t;

void          gw_log_set_level(gw_log_level_t level);
gw_log_level_t gw_log_get_level(void);
bool          gw_log_enabled(gw_log_level_t level);

/* 统一日志出口：输出 "级别/标签/时间戳" 前缀, 时间戳来自端口层毫秒计数 */
void gw_log_printf(gw_log_level_t level, const char *tag, const char *fmt, ...);

#define GW_LOGE(tag, ...) gw_log_printf(GW_LOG_ERROR, tag, __VA_ARGS__)
#define GW_LOGW(tag, ...) gw_log_printf(GW_LOG_WARN,  tag, __VA_ARGS__)
#define GW_LOGI(tag, ...) gw_log_printf(GW_LOG_INFO,  tag, __VA_ARGS__)
#define GW_LOGD(tag, ...) gw_log_printf(GW_LOG_DEBUG, tag, __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* GW_LOG_H */
