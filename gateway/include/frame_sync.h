/*
 * frame_sync.h - 字节流帧同步状态机
 *
 * 解决的问题（简历"用帧同步状态机解决粘包、分包与丢帧"）：
 *   串口/CAN 是字节流, 上层拿到的是"不定长、可能粘连、可能被切断、
 *   可能混入噪声"的字节块。本模块把字节流还原成有序的候选帧, 并
 *   对无法还原的部分给出明确的错误分类, 供总线健康度模块分级处理。
 *
 * 判定规则（按优先级）：
 *   1) CRC 通过 -> 立即判定为一帧（不依赖静默间隔, 因此天生能拆粘包）；
 *   2) 静默间隔 T3.5 到达且 CRC 不通过 -> 上报 CRC 错或残帧；
 *   3) 长度超过最大 ADU -> 超长帧, 丢弃并进入重同步；
 *   4) 出错后逐字节滑动重同步（每次丢 1 字节重新校验）,
 *      这样"噪声字节 + 正确帧"、"帧内某字节被改写"都能自愈。
 *
 * 这样设计的好处是：不把"3.5 字符静默"当作唯一判据。现场上位机
 * 用 USB 转串口时静默间隔经常被驱动打散, 只靠定时器判帧会大面积
 * 丢帧；而 CRC 优先的判据在这种情况下仍然稳定。
 */
#ifndef GW_FRAME_SYNC_H
#define GW_FRAME_SYNC_H

#include "gw_types.h"
#include "gateway_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GW_FS_IDLE      = 0,   /* 等待帧起始                              */
    GW_FS_RECEIVING,       /* 正在累积字节                            */
    GW_FS_RESYNC           /* 出错后滑动重同步                        */
} gw_fs_state_t;

typedef enum {
    GW_FS_EV_NONE = 0,
    GW_FS_EV_FRAME,        /* 完整且 CRC 正确                         */
    GW_FS_EV_CRC_ERROR,    /* 长度匹配但 CRC 校验失败                 */
    GW_FS_EV_SHORT_FRAME,  /* 静默到达时帧不完整（丢字节 / 半包超时） */
    GW_FS_EV_OVERSIZE,     /* 超过最大 ADU 长度                       */
    GW_FS_EV_RESYNC        /* 本次丢弃了若干噪声字节后重新对齐         */
} gw_fs_event_kind_t;

const char *gw_fs_event_str(gw_fs_event_kind_t k);

typedef struct {
    gw_fs_event_kind_t kind;
    const uint8_t     *data;         /* 指向帧同步内部缓冲, 回调返回后失效 */
    uint32_t           len;          /* 帧长度（含 CRC）                   */
    uint32_t           timestamp_us; /* 帧结束时刻                          */
    uint32_t           dropped;      /* 本次事件前丢弃的字节数（重同步量）  */
} gw_fs_event_t;

typedef void (*gw_fs_event_cb)(void *ctx, const gw_fs_event_t *ev);

/* 期望长度推断函数：返回 0 表示"尚无法判断", 否则返回该帧的完整长度。
 * Modbus RTU 用 gw_mb_rtu_expected_len 传入；自定义协议可以传 NULL,
 * 此时只依赖 CRC 判据。 */
typedef uint32_t (*gw_fs_len_fn)(const uint8_t *buf, uint32_t len);

/* 帧完整性校验函数。为 NULL 时默认使用 CRC16/Modbus（本工程所有
 * 串行/管道链路的自定义帧都带 CRC16 尾, 因此默认值即生产值）。 */
typedef bool (*gw_fs_crc_fn)(const uint8_t *buf, uint32_t len);

typedef struct {
    uint8_t        *buf;
    uint32_t        cap;
    uint32_t        len;
    uint32_t        t35_us;         /* 帧间静默门限（T3.5）             */
    uint32_t        last_byte_us;
    uint32_t        frame_start_us;
    gw_fs_state_t   state;
    gw_fs_len_fn    len_fn;
    gw_fs_crc_fn    crc_fn;
    gw_fs_event_cb  cb;
    void           *cb_ctx;

    /* 统计（诊断线程上报, 也是测试断言依据） */
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t short_frames;
    uint32_t oversize;
    uint32_t resync_count;
    uint32_t bytes_in;
    uint32_t garbage_bytes;
} gw_frame_sync_t;

/* 精简的事件汇总, 便于测试一次性断言全部场景 */
typedef struct {
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t short_frames;
    uint32_t oversize;
    uint32_t resync_count;
    uint32_t bytes_in;
    uint32_t garbage_bytes;
} gw_frame_sync_stats_t;

int  gw_frame_sync_init(gw_frame_sync_t *fs, uint8_t *buf, uint32_t cap,
                        uint32_t t35_us, gw_fs_len_fn len_fn,
                        gw_fs_event_cb cb, void *cb_ctx);
/* 覆盖默认的 CRC 校验函数（默认即 CRC16/Modbus） */
void gw_frame_sync_set_crc_fn(gw_frame_sync_t *fs, gw_fs_crc_fn crc_fn);
void gw_frame_sync_reset(gw_frame_sync_t *fs);

/* 喂入一段字节流（可以是半个帧、多个粘连帧或含噪声的字节块） */
void gw_frame_sync_feed(gw_frame_sync_t *fs, const uint8_t *data, uint32_t len,
                        uint32_t timestamp_us);

/* 周期调用：处理"帧尾静默到达"这一事件（丢帧/半包超时的判定） */
void gw_frame_sync_tick(gw_frame_sync_t *fs, uint32_t timestamp_us);

void gw_frame_sync_get_stats(const gw_frame_sync_t *fs, gw_frame_sync_stats_t *out);
gw_fs_state_t gw_frame_sync_state(const gw_frame_sync_t *fs);

#ifdef __cplusplus
}
#endif

#endif /* GW_FRAME_SYNC_H */
