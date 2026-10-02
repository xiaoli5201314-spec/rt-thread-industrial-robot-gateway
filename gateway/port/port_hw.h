/*
 * port_hw.h - 硬件相关操作的抽象层
 *
 * 把"和板子打交道"的部分单独隔离出来：
 *   - 物理链路 (RS485 UART / TCP socket / CAN 控制器)
 *   - 非易失参数区 (NOR Flash / EEPROM, 用于掉电保存)
 *   - 硬件故障标志 (总线短路、过流、反接等由硬件比较器给出的信号)
 *
 * PC 端口用回环链路、管道、文件模拟；目标板端口对接 RT-Thread 的
 * serial / socket / spi 设备框架。
 */
#ifndef GW_PORT_HW_H
#define GW_PORT_HW_H

#include "gw_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* 物理链路                                                            */
/* ------------------------------------------------------------------ */
typedef struct gw_link gw_link_t;

struct gw_link {
    const char *name;
    /* 打开链路, cfg 语义由具体实现解释（如 "127.0.0.1:15020" 或 "/dev/ttyS1"） */
    int  (*open)(gw_link_t *l, const char *cfg);
    int  (*send)(gw_link_t *l, const uint8_t *data, size_t len, int32_t timeout_ms);
    /* 返回实际读到的字节数(>0)、GW_ERR_TIMEOUT、或其它负错误码 */
    int  (*recv)(gw_link_t *l, uint8_t *buf, size_t cap, int32_t timeout_ms);
    void (*close)(gw_link_t *l);
    void *priv;
};

int  gw_link_open(gw_link_t *l, const char *cfg);
int  gw_link_send(gw_link_t *l, const uint8_t *data, size_t len, int32_t timeout_ms);
int  gw_link_recv(gw_link_t *l, uint8_t *buf, size_t cap, int32_t timeout_ms);
void gw_link_close(gw_link_t *l);

/*
 * 回环链路：把"从站"以回调形式接在网关内部。
 * 单元测试用它构造可控从站；gw_host_sim --inproc 也用它。
 *   slave_fn(ctx, req, req_len, resp, resp_cap) -> 响应字节数, 0 表示不应答
 * slave_fn 由调用者在回调内部决定是否注入 CRC 错 / 静默 / 异常码。
 */
typedef int (*gw_link_slave_fn)(void *ctx, const uint8_t *req, size_t req_len,
                                uint8_t *resp, size_t resp_cap);
gw_link_t *gw_link_loopback_create(gw_link_slave_fn slave_fn, void *ctx);

/* stdio 链路：stdin 收、stdout 发, 用于与 Python 模拟从站做管道对接 */
gw_link_t *gw_link_stdio_create(void);

/* TCP 链路：listen 模式（网关做服务端）或 connect 模式（网关做客户端） */
gw_link_t *gw_link_tcp_listen_create(void);
gw_link_t *gw_link_tcp_connect_create(void);

/* CAN 链路（PC 端口为桩实现, 目标板对接 RT-Thread CAN 设备） */
gw_link_t *gw_link_can_create(void);

/* ------------------------------------------------------------------ */
/* 非易失存储（掉电参数保存）                                          */
/* ------------------------------------------------------------------ */
#define GW_NV_TOTAL_SIZE   4096u     /* 参数区总容量：两个 2KB 页轮换     */

int gw_nv_read(uint32_t offset, uint8_t *buf, size_t len);
int gw_nv_write(uint32_t offset, const uint8_t *buf, size_t len);
int gw_nv_erase(uint32_t offset, size_t len);

/* 测试钩子：模拟写入过程中掉电（只写一半） */
void gw_nv_simulate_torn_write(bool enable);

/* 测试钩子：统计擦写次数, 用于说明轮换策略降低单页磨损 */
uint32_t gw_nv_get_erase_count(void);
uint32_t gw_nv_get_write_count(void);

/* ------------------------------------------------------------------ */
/* 硬件故障信号（来自硬件比较器 / 驱动芯片 FAULT 引脚）                */
/* ------------------------------------------------------------------ */
typedef enum {
    GW_HW_FAULT_NONE        = 0x00,
    GW_HW_FAULT_RS485_SHORT = 0x01,  /* RS485 A/B 短路或总线被拉死         */
    GW_HW_FAULT_CAN_BUS_OFF = 0x02,  /* CAN 控制器进入 Bus-Off             */
    GW_HW_FAULT_CAN_OVERCUR = 0x04,  /* CAN 收发器过流 / 显性超时          */
    GW_HW_FAULT_PWR_UVLO    = 0x08,  /* 电源欠压（掉电预警中断）           */
    GW_HW_FAULT_REVERSE_POL = 0x10   /* 输入反接保护动作                   */
} gw_hw_fault_t;

uint32_t gw_hw_get_faults(void);
void     gw_hw_clear_faults(uint32_t mask);
/* 测试注入：端口层提供, 目标板上由 GPIO 中断置位 */
void     gw_hw_inject_fault(uint32_t mask);

/* 掉电预警：电源 UVLO 比较器拉低时置位, 服务层据此立刻保存参数 */
bool gw_hw_power_fail_pending(void);
void gw_hw_inject_power_fail(bool pending);

#ifdef __cplusplus
}
#endif

#endif /* GW_PORT_HW_H */
