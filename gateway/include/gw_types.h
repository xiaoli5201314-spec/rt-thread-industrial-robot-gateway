/*
 * gw_types.h - 工业通信网关公共类型定义
 *
 * 本文件只依赖 C99 标准库，不依赖任何 RTOS / 厂商 SDK，
 * 以便同一套协议与模型代码可以同时在 PC 仿真环境与 RT-Thread 目标板上编译。
 */
#ifndef GW_TYPES_H
#define GW_TYPES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* 编译期辅助宏                                                        */
/* ------------------------------------------------------------------ */
#define GW_ARRAY_SIZE(a)      (sizeof(a) / sizeof((a)[0]))
#define GW_MIN(a, b)          (((a) < (b)) ? (a) : (b))
#define GW_MAX(a, b)          (((a) > (b)) ? (a) : (b))
#define GW_UNUSED(x)          ((void)(x))
#define GW_ALIGN_UP(x, a)     (((x) + ((a) - 1u)) & ~((a) - 1u))

/* 字符串化：用于把编译期常量拼进日志 */
#define GW_STR_(x)            #x
#define GW_STR(x)             GW_STR_(x)

/* ------------------------------------------------------------------ */
/* 统一返回码                                                          */
/*                                                                     */
/* 约定：负数一律为错误，0 为成功；所有可能失败的接口都返回 gw_status_t。 */
/* 关键路径上不用 errno 传播，避免 RTOS 与 PC 端语义不一致。            */
/* ------------------------------------------------------------------ */
typedef enum {
    GW_OK              = 0,   /* 成功                                        */
    GW_ERR             = -1,  /* 未分类错误                                  */
    GW_ERR_PARAM       = -2,  /* 入参非法                                    */
    GW_ERR_TIMEOUT     = -3,  /* 超时                                        */
    GW_ERR_CRC         = -4,  /* CRC 校验失败                                */
    GW_ERR_STATE       = -5,  /* 状态机不允许该操作                          */
    GW_ERR_NOMEM       = -6,  /* 内存池耗尽                                  */
    GW_ERR_IO          = -7,  /* 物理层读写失败                              */
    GW_ERR_NOT_FOUND   = -8,  /* 对象不存在                                  */
    GW_ERR_FULL        = -9,  /* 队列 / 缓冲区满                             */
    GW_ERR_EMPTY       = -10, /* 队列 / 缓冲区空                             */
    GW_ERR_BUSY        = -11, /* 资源被占用                                  */
    GW_ERR_EXCEPTION   = -12, /* 从站返回协议异常响应                        */
    GW_ERR_UNSUPPORTED = -13, /* 功能未实现 / 器件不支持                     */
    GW_ERR_OVERFLOW    = -14, /* 计数或缓冲区溢出                            */
    GW_ERR_NOT_READY   = -15  /* 设备尚未就绪 / 尚未隔离完成                 */
} gw_status_t;

/* 返回码转可读字符串（日志与测试断言使用） */
const char *gw_status_str(gw_status_t st);

/* ------------------------------------------------------------------ */
/* 协议 / 设备 / 总线枚举                                              */
/* ------------------------------------------------------------------ */
typedef enum {
    GW_PROTO_MODBUS_RTU = 0,   /* Modbus RTU  (RS485 半双工, 带 CRC16)     */
    GW_PROTO_MODBUS_TCP,       /* Modbus TCP  (MBAP 头, 无 CRC)            */
    GW_PROTO_CANOPEN,          /* CANopen     (CAN 2.0B, 11/29 位 ID)      */
    GW_PROTO_DEVICENET,        /* DeviceNet   (基于 CAN 的应用层映射)      */
    GW_PROTO_ETHERNET_IP,      /* EtherNet/IP (CIP 显式/隐式报文)          */
    GW_PROTO_COUNT
} gw_protocol_t;

const char *gw_protocol_str(gw_protocol_t p);

/* 设备在网关侧的状态机 */
typedef enum {
    GW_DEV_UNKNOWN  = 0,   /* 刚注册, 尚未通信                            */
    GW_DEV_ONLINE,         /* 心跳正常, 轮询成功                          */
    GW_DEV_DEGRADED,       /* 有偶发错误但仍在通信                        */
    GW_DEV_OFFLINE,        /* 心跳超时 / 连续轮询失败                     */
    GW_DEV_ISOLATED,       /* 被总线健康管理隔离, 暂停轮询                */
    GW_DEV_STATE_COUNT
} gw_dev_state_t;

const char *gw_dev_state_str(gw_dev_state_t s);

/* 总线整体健康度分级（可靠性设计核心枚举） */
typedef enum {
    GW_BUS_OK       = 0,   /* 正常                                        */
    GW_BUS_WARN,           /* 偶发错误, 仅计数告警                        */
    GW_BUS_DEGRADED,       /* 错误率超阈值, 降速 / 减少轮询               */
    GW_BUS_ISOLATED,       /* 隔离问题设备, 其余设备继续工作              */
    GW_BUS_FAULT           /* 总线级故障(短路), 停止发送并请求恢复        */
} gw_bus_state_t;

const char *gw_bus_state_str(gw_bus_state_t s);

/* ------------------------------------------------------------------ */
/* 大端序读写（Modbus / CANopen 均为大端）                             */
/* ------------------------------------------------------------------ */
static inline uint16_t gw_read_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static inline void gw_write_be16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFFu);
}

static inline uint32_t gw_read_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

static inline void gw_write_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

#ifdef __cplusplus
}
#endif

#endif /* GW_TYPES_H */
