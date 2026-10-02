/*
 * test_frame_sync.c - 帧同步状态机测试
 *
 * 验收标准要求 4 种场景全部有测试且通过：
 *   1) 分包（半包）  : 一帧被切成多段, 段间隔小于 T3.5 -> 必须重组成一帧
 *   2) 粘包          : 多帧连续到达（无静默间隔） -> 必须按 CRC 正确切开
 *   3) 错帧          : 帧内某字节被改写 -> 必须报 CRC 错, 且滑动重同步后
 *                      仍能捕获紧随其后的正确帧
 *   4) 丢帧          : 帧中间少了字节, 随后出现静默 -> 必须报残帧,
 *                      且不得误报成正常帧；后续正常帧不受影响
 */
#include "test_util.h"
#include "frame_sync.h"
#include "modbus_rtu.h"
#include "gw_crc.h"

#include <string.h>

/* 事件采集器 */
#define EV_MAX 32
typedef struct {
    gw_fs_event_kind_t kind[EV_MAX];
    uint32_t           len[EV_MAX];
    uint8_t            data[EV_MAX][GW_MAX_ADU_RTU];
    uint32_t           dropped[EV_MAX];
    int                count;
} ev_log_t;

static ev_log_t g_ev;

static void ev_cb(void *ctx, const gw_fs_event_t *ev)
{
    ev_log_t *log = (ev_log_t *)ctx;

    if (log->count >= EV_MAX) {
        return;
    }
    log->kind[log->count]    = ev->kind;
    log->len[log->count]     = ev->len;
    log->dropped[log->count] = ev->dropped;
    if (ev->len <= GW_MAX_ADU_RTU) {
        memcpy(log->data[log->count], ev->data, ev->len);
    }
    log->count++;
}

static int count_kind(const ev_log_t *log, gw_fs_event_kind_t k)
{
    int i;
    int n = 0;
    for (i = 0; i < log->count; i++) {
        if (log->kind[i] == k) {
            n++;
        }
    }
    return n;
}

static int first_index_of(const ev_log_t *log, gw_fs_event_kind_t k)
{
    int i;
    for (i = 0; i < log->count; i++) {
        if (log->kind[i] == k) {
            return i;
        }
    }
    return -1;
}

/* 构造 3 个合法的 FC03 请求帧（长度分别为 8 / 8 / 8） */
static uint32_t build_req(uint8_t *out, uint8_t slave, uint16_t addr, uint16_t qty)
{
    uint32_t len = 0u;
    if (gw_mb_rtu_build_read(slave, GW_MB_FC_READ_HOLDING, addr, qty, out,
                             64u, &len) != GW_OK) {
        return 0u;
    }
    return len;
}

void test_frame_sync(void)
{
    uint8_t  buf[GW_FRAME_SYNC_BUF];
    gw_frame_sync_t fs;
    uint8_t  f1[16];
    uint8_t  f2[16];
    uint8_t  f3[16];
    uint32_t l1;
    uint32_t l2;
    uint32_t l3;
    gw_frame_sync_stats_t st;

    l1 = build_req(f1, 0x01u, 0x0000u, 0x000Au);   /* 8 字节 */
    l2 = build_req(f2, 0x02u, 0x0010u, 0x0004u);   /* 8 字节 */
    l3 = build_req(f3, 0x03u, 0x0020u, 0x0002u);   /* 8 字节 */
    GW_ASSERT_EQ_INT(l1, 8);
    GW_ASSERT_EQ_INT(l2, 8);
    GW_ASSERT_EQ_INT(l3, 8);

    /* ---------------- 场景 1：分包（半包） ---------------- */
    GW_SUITE("帧同步状态机");
    GW_CASE("场景 1/4 分包：一帧分 3 段到达（间隔 < T3.5）-> 重组为 1 帧");
    {
        memset(&g_ev, 0, sizeof(g_ev));
        GW_ASSERT_EQ_INT(gw_frame_sync_init(&fs, buf, sizeof(buf), GW_T35_US,
                                            gw_mb_rtu_expected_len, ev_cb,
                                            &g_ev), GW_OK);
        gw_frame_sync_feed(&fs, f1, 3u, 1000u);          /* 前 3 字节 */
        gw_frame_sync_feed(&fs, f1 + 3, 3u, 2000u);      /* 再 3 字节 */
        GW_ASSERT_EQ_INT(g_ev.count, 0);                 /* 还没收全 */
        GW_ASSERT_EQ_INT((int)gw_frame_sync_state(&fs), (int)GW_FS_RECEIVING);
        gw_frame_sync_feed(&fs, f1 + 6, 2u, 3000u);      /* 最后 2 字节 */
        GW_ASSERT_EQ_INT(g_ev.count, 1);
        GW_ASSERT_EQ_INT(g_ev.kind[0], GW_FS_EV_FRAME);
        GW_ASSERT_EQ_INT(g_ev.len[0], 8);
        GW_ASSERT_EQ_MEM(g_ev.data[0], f1, 8u);
        gw_frame_sync_get_stats(&fs, &st);
        GW_ASSERT_EQ_INT(st.frames_ok, 1u);
        GW_ASSERT_EQ_INT(st.crc_errors, 0u);
        GW_ASSERT_EQ_INT(st.short_frames, 0u);
    }

    /* ---------------- 场景 2：粘包 ---------------- */
    GW_CASE("场景 2/4 粘包：3 帧一次到达（无静默）-> 必须切成 3 帧且顺序正确");
    {
        uint8_t stream[64];
        memset(&g_ev, 0, sizeof(g_ev));
        GW_ASSERT_EQ_INT(gw_frame_sync_init(&fs, buf, sizeof(buf), GW_T35_US,
                                            gw_mb_rtu_expected_len, ev_cb,
                                            &g_ev), GW_OK);
        memcpy(&stream[0], f1, l1);
        memcpy(&stream[l1], f2, l2);
        memcpy(&stream[l1 + l2], f3, l3);
        gw_frame_sync_feed(&fs, stream, l1 + l2 + l3, 5000u);

        GW_ASSERT_EQ_INT(g_ev.count, 3);
        GW_ASSERT_EQ_INT(count_kind(&g_ev, GW_FS_EV_FRAME), 3);
        GW_ASSERT_EQ_MEM(g_ev.data[0], f1, l1);
        GW_ASSERT_EQ_MEM(g_ev.data[1], f2, l2);
        GW_ASSERT_EQ_MEM(g_ev.data[2], f3, l3);
        gw_frame_sync_get_stats(&fs, &st);
        GW_ASSERT_EQ_INT(st.crc_errors, 0u);
        GW_ASSERT_EQ_INT(st.garbage_bytes, 0u);
        GW_ASSERT_EQ_INT((int)gw_frame_sync_state(&fs), (int)GW_FS_IDLE);
    }

    /* ---------------- 场景 3：错帧 ---------------- */
    GW_CASE("场景 3/4 错帧：帧内字节被改写 -> 报 CRC 错, 其后正确帧仍被捕获");
    {
        uint8_t bad[16];
        uint8_t stream[64];
        int     crc_idx;

        memset(&g_ev, 0, sizeof(g_ev));
        GW_ASSERT_EQ_INT(gw_frame_sync_init(&fs, buf, sizeof(buf), GW_T35_US,
                                            gw_mb_rtu_expected_len, ev_cb,
                                            &g_ev), GW_OK);
        memcpy(bad, f1, l1);
        bad[4] ^= 0x5Au;                    /* 破坏数据字节 -> CRC 必然不过 */

        memcpy(&stream[0], bad, l1);
        memcpy(&stream[l1], f2, l2);
        gw_frame_sync_feed(&fs, stream, l1 + l2, 9000u);

        gw_frame_sync_get_stats(&fs, &st);
        GW_ASSERT(st.crc_errors >= 1u);                 /* 必须报错, 不能静默吞掉 */
        GW_ASSERT_EQ_INT(count_kind(&g_ev, GW_FS_EV_FRAME), 1);
        crc_idx = first_index_of(&g_ev, GW_FS_EV_CRC_ERROR);
        GW_ASSERT(crc_idx >= 0);
        /* 重同步后捕获到的必须是第二个（正确的）帧 */
        GW_ASSERT_EQ_MEM(g_ev.data[g_ev.count - 1], f2, l2);
        GW_ASSERT_EQ_INT(g_ev.kind[g_ev.count - 1], GW_FS_EV_FRAME);
    }

    GW_CASE("场景 3b 错帧：纯噪声字节流不会产生任何合法帧");
    {
        uint8_t noise[64];
        int     i;
        memset(&g_ev, 0, sizeof(g_ev));
        memset(noise, 0x5A, sizeof(noise));
        GW_ASSERT_EQ_INT(gw_frame_sync_init(&fs, buf, sizeof(buf), GW_T35_US,
                                            gw_mb_rtu_expected_len, ev_cb,
                                            &g_ev), GW_OK);
        for (i = 0; i < 8; i++) {
            gw_frame_sync_feed(&fs, noise, 8u, (uint32_t)(10000u + i * 50u));
            gw_frame_sync_tick(&fs, (uint32_t)(10000u + i * 50u + 8000u));
        }
        GW_ASSERT_EQ_INT(count_kind(&g_ev, GW_FS_EV_FRAME), 0);
        gw_frame_sync_get_stats(&fs, &st);
        GW_ASSERT(st.crc_errors + st.short_frames > 0u);
    }

    /* ---------------- 场景 4：丢帧 ---------------- */
    GW_CASE("场景 4/4 丢帧：帧中间少 1 字节后出现静默 -> 报残帧, 不误判为正常帧");
    {
        uint8_t truncated[16];
        memset(&g_ev, 0, sizeof(g_ev));
        GW_ASSERT_EQ_INT(gw_frame_sync_init(&fs, buf, sizeof(buf), GW_T35_US,
                                            gw_mb_rtu_expected_len, ev_cb,
                                            &g_ev), GW_OK);
        /* 去掉第 4 个字节（数量低字节），得到 7 字节的不完整帧 */
        memcpy(truncated, f1, 3u);
        memcpy(truncated + 3, f1 + 4, l1 - 4u);
        gw_frame_sync_feed(&fs, truncated, l1 - 1u, 20000u);
        GW_ASSERT_EQ_INT(g_ev.count, 0);           /* 还不足以判帧 */
        /* T3.5 静默到达：本帧被判为残帧 */
        gw_frame_sync_tick(&fs, 20000u + GW_T35_US + 100u);
        GW_ASSERT_EQ_INT(g_ev.count, 1);
        GW_ASSERT_EQ_INT(g_ev.kind[0], GW_FS_EV_SHORT_FRAME);
        GW_ASSERT_EQ_INT(count_kind(&g_ev, GW_FS_EV_FRAME), 0);   /* 不得误报 */
        gw_frame_sync_get_stats(&fs, &st);
        GW_ASSERT_EQ_INT(st.short_frames, 1u);
        GW_ASSERT_EQ_INT(st.frames_ok, 0u);

        /* 残帧之后紧跟一帧正确的：必须被正常解析 */
        gw_frame_sync_feed(&fs, f3, l3, 20000u + GW_T35_US + 2000u);
        GW_ASSERT_EQ_INT(count_kind(&g_ev, GW_FS_EV_FRAME), 1);
        GW_ASSERT_EQ_MEM(g_ev.data[g_ev.count - 1], f3, l3);
    }

    GW_CASE("综合：分包 + 粘包 + 错帧 + 丢帧混合字节流, 计数与顺序全部正确");
    {
        uint8_t stream[128];
        uint32_t n = 0u;
        /* 1) f1 分两段（分包） */
        memcpy(&stream[n], f1, 4u); n += 4u;
        memset(&g_ev, 0, sizeof(g_ev));
        GW_ASSERT_EQ_INT(gw_frame_sync_init(&fs, buf, sizeof(buf), GW_T35_US,
                                            gw_mb_rtu_expected_len, ev_cb,
                                            &g_ev), GW_OK);
        gw_frame_sync_feed(&fs, stream, n, 30000u);
        gw_frame_sync_feed(&fs, f1 + 4u, l1 - 4u, 30000u + 500u);
        /* 2) f2 立即跟上（粘包） */
        gw_frame_sync_feed(&fs, f2, l2, 30000u + 600u);
        /* 3) 一帧被破坏 */
        {
            uint8_t bad[16];
            memcpy(bad, f3, l3);
            bad[1] ^= 0x01u;
            gw_frame_sync_feed(&fs, bad, l3, 30000u + 700u);
        }
        /* 4) 一帧丢字节 + 静默 */
        {
            uint8_t trunc[16];
            memcpy(trunc, f1, 3u);
            memcpy(trunc + 3u, f1 + 4u, l1 - 4u);
            gw_frame_sync_feed(&fs, trunc, l1 - 1u, 30000u + 900u);
            gw_frame_sync_tick(&fs, 30000u + 900u + GW_T35_US + 50u);
        }
        /* 5) 最后再来一帧正确的 */
        gw_frame_sync_feed(&fs, f3, l3, 30000u + 30000u);

        GW_ASSERT_EQ_INT(count_kind(&g_ev, GW_FS_EV_FRAME), 3);      /* f1 f2 f3 */
        GW_ASSERT_EQ_INT(count_kind(&g_ev, GW_FS_EV_CRC_ERROR), 1);
        GW_ASSERT_EQ_INT(count_kind(&g_ev, GW_FS_EV_SHORT_FRAME), 1);
        GW_ASSERT_EQ_MEM(g_ev.data[0], f1, l1);
        GW_ASSERT_EQ_MEM(g_ev.data[1], f2, l2);
        GW_ASSERT_EQ_MEM(g_ev.data[g_ev.count - 1], f3, l3);
    }

    GW_CASE("纯噪声（功能码非法）不会堆积：逐字节被丢弃并计入垃圾字节");
    {
        uint8_t junk[64];
        int     i;
        memset(&g_ev, 0, sizeof(g_ev));
        memset(junk, 0xA5, sizeof(junk));     /* 0xA5 & 0x7F = 0x25, 非法功能码 */
        GW_ASSERT_EQ_INT(gw_frame_sync_init(&fs, buf, 64u, GW_T35_US, NULL,
                                            ev_cb, &g_ev), GW_OK);
        for (i = 0; i < 40; i++) {
            gw_frame_sync_feed(&fs, junk, sizeof(junk), (uint32_t)(40000u + i * 10u));
        }
        gw_frame_sync_get_stats(&fs, &st);
        GW_ASSERT_EQ_INT(count_kind(&g_ev, GW_FS_EV_FRAME), 0);
        GW_ASSERT(st.resync_count > 0u);
        GW_ASSERT(st.garbage_bytes > 0u);
        GW_ASSERT(fs.len <= 2u);              /* 缓冲不会被噪声撑大 */
    }

    GW_CASE("超长帧保护：功能码合法但内容始终不成帧 -> 触发 OVERSIZE 且不越界");
    {
        uint8_t junk[80];
        int     i;
        memset(&g_ev, 0, sizeof(g_ev));
        memset(junk, 0x00, sizeof(junk));
        junk[0] = 0x01;                       /* 从站地址 */
        junk[1] = 0x03;                       /* 合法功能码 -> 头部可行, 会持续累积 */
        GW_ASSERT_EQ_INT(gw_frame_sync_init(&fs, buf, 64u, GW_T35_US, NULL,
                                            ev_cb, &g_ev), GW_OK);
        for (i = 0; i < 8; i++) {
            gw_frame_sync_feed(&fs, junk, sizeof(junk), (uint32_t)(50000u + i * 10u));
        }
        gw_frame_sync_get_stats(&fs, &st);
        GW_ASSERT_EQ_INT(count_kind(&g_ev, GW_FS_EV_FRAME), 0);
        GW_ASSERT(st.oversize > 0u);
        GW_ASSERT(fs.len <= 64u);
    }
}
