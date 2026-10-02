/*
 * canopen.h - CAN / CANopen 帧解析与对象字典映射
 *
 * 覆盖简历技术点：
 *   - CAN 帧解析（11 位标准帧 / 29 位扩展帧, DLC 0..8）
 *   - CANopen 预定义连接集 COB-ID 解码（NMT/SYNC/EMCY/TPDO/RPDO/SDO/心跳）
 *   - SDO 加速传输与分段传输（对象字典读写）
 *   - PDO 映射解码（把 8 字节过程数据按映射表拆进对象字典）
 *   - NMT 状态机与心跳消费（设备保活）
 *
 * 对象字典采用"声明式表 + 运行期存放点"的方式：字典项本身是 const,
 * 指向的实际存储由设备模型提供, 便于上位机在线读写。
 */
#ifndef GW_CANOPEN_H
#define GW_CANOPEN_H

#include "gw_types.h"
#include "gateway_config.h"
#include "port_rtos.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================== */
/* CAN 帧                                                             */
/* ================================================================== */
#define GW_CAN_MAX_DLC     8u

typedef struct {
    uint32_t id;            /* 11 位或 29 位标识符                       */
    bool     extended;      /* true = 29 位扩展帧                        */
    bool     rtr;           /* 远程帧                                  */
    uint8_t  dlc;           /* 数据长度 0..8                           */
    uint8_t  data[GW_CAN_MAX_DLC];
    uint32_t timestamp_ms;
} gw_can_frame_t;

/* ================================================================== */
/* CANopen 预定义连接集                                                */
/* ================================================================== */
typedef enum {
    GW_COB_NMT        = 0x000,   /* NMT 网络管理                */
    GW_COB_SYNC       = 0x080,   /* SYNC 同步对象（恰好 0x080）  */
    /* EMCY 实际为 0x080 + node(1..127), 与 SYNC 共享基址。
     * 这里取 0x081 作为"代表值", 仅用于类型标签与字符串转换；
     * 真正的判定必须走 gw_canopen_decode_cob()（按基址+节点号解码）。 */
    GW_COB_EMCY       = 0x081,
    GW_COB_TIME       = 0x100,   /* TIME 时间戳                 */
    GW_COB_TPDO1      = 0x180,
    GW_COB_RPDO1      = 0x200,
    GW_COB_TPDO2      = 0x280,
    GW_COB_RPDO2      = 0x300,
    GW_COB_TPDO3      = 0x380,
    GW_COB_RPDO3      = 0x400,
    GW_COB_TPDO4      = 0x480,
    GW_COB_RPDO4      = 0x500,
    GW_COB_SDO_TX     = 0x580,   /* 从站 -> 主站（上传响应）     */
    GW_COB_SDO_RX     = 0x600,   /* 主站 -> 从站（下载请求）     */
    GW_COB_HEARTBEAT  = 0x700,   /* 心跳 / 节点保护             */
    GW_COB_LSS_TX     = 0x7E4,
    GW_COB_LSS_RX     = 0x7E5,
    GW_COB_UNKNOWN    = 0xFFFF
} gw_canopen_cob_t;

const char *gw_canopen_cob_str(gw_canopen_cob_t c);

/* 由 11 位 COB-ID 解码出功能类型与节点号 */
int gw_canopen_decode_cob(uint32_t cob_id, gw_canopen_cob_t *type,
                          uint8_t *node_id);

/* ================================================================== */
/* NMT                                                                */
/* ================================================================== */
#define GW_NMT_CMD_START       0x01u
#define GW_NMT_CMD_STOP        0x02u
#define GW_NMT_CMD_PREOP       0x80u
#define GW_NMT_CMD_RESET_NODE  0x81u
#define GW_NMT_CMD_RESET_COMM  0x82u

/* NMT 状态机状态（心跳数据字节 / 状态查询返回） */
#define GW_NMT_STATE_BOOTUP   0x00u
#define GW_NMT_STATE_STOPPED  0x04u
#define GW_NMT_STATE_OPERATIONAL 0x05u
#define GW_NMT_STATE_PREOP    0x7Fu

int  gw_canopen_nmt_build(uint8_t command, uint8_t node_id, gw_can_frame_t *out);
int  gw_canopen_nmt_parse(const gw_can_frame_t *f, uint8_t *command, uint8_t *node_id);

/* ================================================================== */
/* 心跳消费（设备保活）                                                */
/* ================================================================== */
#define GW_CANOPEN_MAX_NODES  32u

typedef struct {
    uint8_t  node_id;
    bool     active;
    uint8_t  state;              /* GW_NMT_STATE_*                     */
    uint32_t last_seen_ms;
    uint32_t timeout_ms;         /* 该节点的心跳超时                    */
    uint32_t heartbeat_count;
    uint32_t timeout_count;
    bool     online;
} gw_canopen_hb_node_t;

typedef struct {
    gw_canopen_hb_node_t nodes[GW_CANOPEN_MAX_NODES];
    uint32_t             online_count;
    uint32_t             total_timeouts;
} gw_canopen_hb_t;

int  gw_canopen_hb_init(gw_canopen_hb_t *hb);
int  gw_canopen_hb_register(gw_canopen_hb_t *hb, uint8_t node_id, uint32_t timeout_ms);
/* 收到心跳帧：返回受影响节点下标, 负值为错误 */
int  gw_canopen_hb_feed(gw_canopen_hb_t *hb, const gw_can_frame_t *f, uint32_t now_ms);
/* 周期调用：检查全部节点是否超时, 返回本次新超时的节点数 */
int  gw_canopen_hb_check(gw_canopen_hb_t *hb, uint32_t now_ms);
int  gw_canopen_hb_find(gw_canopen_hb_t *hb, uint8_t node_id);

/* ================================================================== */
/* 对象字典                                                            */
/* ================================================================== */
typedef enum {
    GW_OD_U8 = 0,
    GW_OD_U16,
    GW_OD_U32,
    GW_OD_I16,
    GW_OD_I32,
    GW_OD_F32,
    GW_OD_STRING
} gw_od_type_t;

#define GW_OD_ACCESS_RO  0x01u
#define GW_OD_ACCESS_WO  0x02u
#define GW_OD_ACCESS_RW  0x03u

typedef struct {
    uint16_t       index;
    uint8_t        subindex;
    gw_od_type_t   type;
    uint8_t        access;
    const char    *name;
    void          *storage;      /* 指向实际变量                        */
    uint32_t       storage_size; /* 字符串类型的容量                    */
} gw_od_entry_t;

typedef struct {
    const gw_od_entry_t *entries;
    uint32_t             count;
} gw_od_t;

const gw_od_entry_t *gw_od_lookup(const gw_od_t *od, uint16_t index, uint8_t subindex);
int  gw_od_read_u32(const gw_od_entry_t *e, uint32_t *out);
int  gw_od_write_u32(const gw_od_entry_t *e, uint32_t value);
uint32_t gw_od_type_size(gw_od_type_t t);
const char *gw_od_type_str(gw_od_type_t t);

/* ================================================================== */
/* SDO                                                                */
/* ================================================================== */
#define GW_SDO_CS_DOWNLOAD_INIT_SEG   0x21u   /* 分段下载, 带总长度       */
#define GW_SDO_CS_DOWNLOAD_INIT_NOSIZE 0x22u
#define GW_SDO_CS_DOWNLOAD_SEG        0x00u
#define GW_SDO_CS_DOWNLOAD_ACK        0x20u
#define GW_SDO_CS_UPLOAD_INIT         0x40u
#define GW_SDO_CS_UPLOAD_RESP_SEG     0x41u
#define GW_SDO_CS_UPLOAD_REQ_SEG      0x60u
#define GW_SDO_CS_ABORT               0x80u

/* 加速传输的 CS 编码（含数据长度与是否带索引） */
#define GW_SDO_CS_DOWNLOAD_EXP_4B     0x23u
#define GW_SDO_CS_DOWNLOAD_EXP_3B     0x27u
#define GW_SDO_CS_DOWNLOAD_EXP_2B     0x2Bu
#define GW_SDO_CS_DOWNLOAD_EXP_1B     0x2Fu
#define GW_SDO_CS_UPLOAD_EXP_4B       0x43u
#define GW_SDO_CS_UPLOAD_EXP_3B       0x47u
#define GW_SDO_CS_UPLOAD_EXP_2B       0x4Bu
#define GW_SDO_CS_UPLOAD_EXP_1B       0x4Fu

typedef struct {
    bool     valid;
    uint16_t index;
    uint8_t  subindex;
    uint32_t value;
    uint8_t  size;               /* 有效字节数 1..4                    */
    bool     aborted;
    uint32_t abort_code;
} gw_sdo_result_t;

/* 构造：主站写对象（加速传输, <=4 字节） */
int gw_sdo_build_download_expedited(uint8_t node_id, uint16_t index,
                                    uint8_t subindex, uint32_t value, uint8_t size,
                                    gw_can_frame_t *out);
/* 构造：主站读对象 */
int gw_sdo_build_upload_request(uint8_t node_id, uint16_t index,
                                uint8_t subindex, gw_can_frame_t *out);
/* 解析：从站侧的下载请求 / 主站侧的上传响应 */
int gw_sdo_parse(const gw_can_frame_t *f, gw_sdo_result_t *out);

/* 分段传输（>4 字节对象, 例如设备名、标定表） */
int gw_sdo_build_initiate_download(uint8_t node_id, uint16_t index,
                                   uint8_t subindex, uint32_t total_size,
                                   gw_can_frame_t *out);
int gw_sdo_build_segment(uint8_t node_id, bool toggle, const uint8_t *data,
                         uint8_t len, gw_can_frame_t *out);
int gw_sdo_parse_segment(const gw_can_frame_t *f, bool *toggle, uint8_t *out,
                         uint8_t *out_len);
int gw_sdo_build_segment_ack(uint8_t node_id, bool toggle, gw_can_frame_t *out);

/* ================================================================== */
/* PDO 映射                                                            */
/* ================================================================== */
#define GW_PDO_MAX_ENTRIES  8u

typedef struct {
    uint16_t index;
    uint8_t  subindex;
    uint8_t  bits;              /* 8/16/32                            */
} gw_pdo_entry_t;

typedef struct {
    uint8_t        cob_id;      /* 11 位 COB-ID                       */
    uint8_t        count;
    gw_pdo_entry_t entries[GW_PDO_MAX_ENTRIES];
} gw_pdo_map_t;

/* 按映射表把过程数据写入对象字典 */
int gw_pdo_decode_to_od(const gw_pdo_map_t *map, const gw_od_t *od,
                        const uint8_t *data, uint32_t len);

/* ================================================================== */
/* EMCY 紧急报文                                                       */
/* ================================================================== */
typedef struct {
    uint8_t  node_id;
    uint16_t error_code;
    uint8_t  error_register;
    uint8_t  vendor_data[5];
} gw_emcy_t;

int gw_canopen_emcy_parse(const gw_can_frame_t *f, gw_emcy_t *out);

/* ================================================================== */
/* CAN 帧解析器（带统计, 供总线健康度使用）                            */
/* ================================================================== */
typedef struct {
    uint32_t frames_total;
    uint32_t frames_nmt;
    uint32_t frames_sync;
    uint32_t frames_emcy;
    uint32_t frames_pdo;
    uint32_t frames_sdo;
    uint32_t frames_heartbeat;
    uint32_t frames_unknown;
    uint32_t frames_bad_dlc;
} gw_canopen_stats_t;

/* 分发一帧：按 COB-ID 归类并返回类型 */
int gw_canopen_dispatch(const gw_can_frame_t *f, gw_canopen_cob_t *type,
                        uint8_t *node_id, gw_canopen_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* GW_CANOPEN_H */
