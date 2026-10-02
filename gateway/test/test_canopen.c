/*
 * test_canopen.c - CAN / CANopen 协议测试
 */
#include "test_util.h"
#include "canopen.h"
#include "port_rtos.h"

#include <string.h>

/* 测试用对象字典存放点 */
static uint16_t s_od_u16   = 0u;
static uint32_t s_od_u32   = 0u;
static uint8_t  s_od_u8    = 0u;
static int16_t  s_od_i16   = 0;

static const gw_od_entry_t s_entries[] = {
    { 0x1000u, 0u, GW_OD_U32, GW_OD_ACCESS_RO, "device_type",     &s_od_u32, 0u },
    { 0x1017u, 0u, GW_OD_U16, GW_OD_ACCESS_RW, "producer_hb_time",&s_od_u16, 0u },
    { 0x6000u, 1u, GW_OD_U8,  GW_OD_ACCESS_RW, "do_ch1",          &s_od_u8,  0u },
    { 0x6401u, 1u, GW_OD_U16, GW_OD_ACCESS_RO, "di_ch1_8",        &s_od_u16, 0u },
    { 0x2000u, 1u, GW_OD_I16, GW_OD_ACCESS_RW, "weld_current",    &s_od_i16, 0u },
};

static const gw_od_t s_od = { s_entries, GW_ARRAY_SIZE(s_entries) };

static void make_frame(gw_can_frame_t *f, uint32_t id, const uint8_t *d, uint8_t dlc)
{
    memset(f, 0, sizeof(*f));
    f->id  = id;
    f->dlc = dlc;
    if ((d != NULL) && (dlc > 0u)) {
        memcpy(f->data, d, dlc);
    }
}

void test_canopen(void)
{
    GW_SUITE("CAN / CANopen");

    GW_CASE("COB-ID 解码：NMT / SYNC / TIME / 心跳 / SDO / PDO 全覆盖");
    {
        gw_canopen_cob_t t;
        uint8_t          node = 0u;
        GW_ASSERT_EQ_INT(gw_canopen_decode_cob(0x000u, &t, &node), GW_OK);
        GW_ASSERT_EQ_INT(t, (int)GW_COB_NMT);
        GW_ASSERT_EQ_INT(gw_canopen_decode_cob(0x080u, &t, &node), GW_OK);
        GW_ASSERT_EQ_INT(t, (int)GW_COB_SYNC);
        GW_ASSERT_EQ_INT(gw_canopen_decode_cob(0x100u, &t, &node), GW_OK);
        GW_ASSERT_EQ_INT(t, (int)GW_COB_TIME);
        GW_ASSERT_EQ_INT(gw_canopen_decode_cob(0x083u, &t, &node), GW_OK);
        GW_ASSERT_EQ_INT(t, (int)GW_COB_EMCY);
        GW_ASSERT_EQ_INT(node, 3);
        GW_ASSERT_EQ_INT(gw_canopen_decode_cob(0x181u, &t, &node), GW_OK);
        GW_ASSERT_EQ_INT(t, (int)GW_COB_TPDO1);
        GW_ASSERT_EQ_INT(node, 1);
        GW_ASSERT_EQ_INT(gw_canopen_decode_cob(0x202u, &t, &node), GW_OK);
        GW_ASSERT_EQ_INT(t, (int)GW_COB_RPDO1);
        GW_ASSERT_EQ_INT(gw_canopen_decode_cob(0x284u, &t, &node), GW_OK);
        GW_ASSERT_EQ_INT(t, (int)GW_COB_TPDO2);
        GW_ASSERT_EQ_INT(gw_canopen_decode_cob(0x305u, &t, &node), GW_OK);
        GW_ASSERT_EQ_INT(t, (int)GW_COB_RPDO2);
        GW_ASSERT_EQ_INT(gw_canopen_decode_cob(0x386u, &t, &node), GW_OK);
        GW_ASSERT_EQ_INT(t, (int)GW_COB_TPDO3);
        GW_ASSERT_EQ_INT(gw_canopen_decode_cob(0x407u, &t, &node), GW_OK);
        GW_ASSERT_EQ_INT(t, (int)GW_COB_RPDO3);
        GW_ASSERT_EQ_INT(gw_canopen_decode_cob(0x588u, &t, &node), GW_OK);
        GW_ASSERT_EQ_INT(t, (int)GW_COB_SDO_TX);
        GW_ASSERT_EQ_INT(node, 8);
        GW_ASSERT_EQ_INT(gw_canopen_decode_cob(0x609u, &t, &node), GW_OK);
        GW_ASSERT_EQ_INT(t, (int)GW_COB_SDO_RX);
        GW_ASSERT_EQ_INT(gw_canopen_decode_cob(0x70Au, &t, &node), GW_OK);
        GW_ASSERT_EQ_INT(t, (int)GW_COB_HEARTBEAT);
        GW_ASSERT_EQ_INT(node, 10);
        GW_ASSERT_EQ_INT(gw_canopen_decode_cob(0x7E5u, &t, &node), GW_OK);
        GW_ASSERT_EQ_INT(t, (int)GW_COB_LSS_RX);
        /* 0x7F0 不属于任何预定义连接集区间 */
        GW_ASSERT_EQ_INT(gw_canopen_decode_cob(0x7F0u, &t, &node), GW_ERR_NOT_FOUND);
        GW_ASSERT_EQ_INT(t, (int)GW_COB_UNKNOWN);
        GW_ASSERT_EQ_INT(gw_canopen_decode_cob(0x1ABCDEFu, &t, &node),
                         GW_ERR_UNSUPPORTED);      /* 29 位扩展帧不走预定义连接集 */
    }

    GW_CASE("CAN 帧分发统计（含 DLC 非法）");
    {
        gw_canopen_stats_t st;
        gw_canopen_cob_t   t;
        uint8_t            n;
        gw_can_frame_t     f;
        uint8_t            d[8] = { 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

        memset(&st, 0, sizeof(st));
        make_frame(&f, 0x000u, d, 2u);
        GW_ASSERT_EQ_INT(gw_canopen_dispatch(&f, &t, &n, &st), GW_OK);
        make_frame(&f, 0x181u, d, 4u);
        GW_ASSERT_EQ_INT(gw_canopen_dispatch(&f, &t, &n, &st), GW_OK);
        make_frame(&f, 0x581u, d, 8u);
        GW_ASSERT_EQ_INT(gw_canopen_dispatch(&f, &t, &n, &st), GW_OK);
        make_frame(&f, 0x701u, d, 1u);
        GW_ASSERT_EQ_INT(gw_canopen_dispatch(&f, &t, &n, &st), GW_OK);
        make_frame(&f, 0x7F0u, d, 1u);            /* 不在任何预定义区间 */
        GW_ASSERT_EQ_INT(gw_canopen_dispatch(&f, &t, &n, &st), GW_ERR_NOT_FOUND);
        {
            uint8_t big[9] = {0};
            make_frame(&f, 0x200u, big, 9u);        /* DLC = 9 非法 */
        }
        GW_ASSERT_EQ_INT(gw_canopen_dispatch(&f, &t, &n, &st), GW_ERR_PARAM);

        GW_ASSERT_EQ_INT(st.frames_total, 5u);
        GW_ASSERT_EQ_INT(st.frames_nmt, 1u);
        GW_ASSERT_EQ_INT(st.frames_pdo, 1u);
        GW_ASSERT_EQ_INT(st.frames_sdo, 1u);
        GW_ASSERT_EQ_INT(st.frames_heartbeat, 1u);
        GW_ASSERT_EQ_INT(st.frames_unknown, 1u);
        GW_ASSERT_EQ_INT(st.frames_bad_dlc, 1u);
    }

    GW_CASE("NMT 命令构造与解析");
    {
        gw_can_frame_t f;
        uint8_t        cmd = 0u;
        uint8_t        node = 0u;
        GW_ASSERT_EQ_INT(gw_canopen_nmt_build(GW_NMT_CMD_START, 5u, &f), GW_OK);
        GW_ASSERT_EQ_INT(f.id, 0x000u);
        GW_ASSERT_EQ_INT(f.dlc, 2u);
        GW_ASSERT_EQ_INT(gw_canopen_nmt_parse(&f, &cmd, &node), GW_OK);
        GW_ASSERT_EQ_INT(cmd, GW_NMT_CMD_START);
        GW_ASSERT_EQ_INT(node, 5u);
        GW_ASSERT_EQ_INT(gw_canopen_nmt_build(GW_NMT_CMD_START, 0x80u, &f),
                         GW_ERR_PARAM);
        GW_ASSERT_EQ_INT(gw_canopen_nmt_build(GW_NMT_CMD_RESET_NODE, 0u, &f), GW_OK);
        GW_ASSERT_EQ_INT(gw_canopen_nmt_parse(&f, &cmd, &node), GW_OK);
        GW_ASSERT_EQ_INT(node, 0u);               /* 0 = 广播 */
    }

    GW_CASE("心跳消费：注册/喂狗/超时转离线/重新上线");
    {
        gw_canopen_hb_t hb;
        gw_can_frame_t  f;
        uint8_t         d[2] = { GW_NMT_STATE_OPERATIONAL, 0u };
        /* 用端口真实时基作基准, 避免与注册时间戳错位导致回绕 */
        uint32_t        t = gw_port_tick_ms() + 1000u;

        GW_ASSERT_EQ_INT(gw_canopen_hb_init(&hb), GW_OK);
        GW_ASSERT_EQ_INT(gw_canopen_hb_register(&hb, 1u, 500u), 0);
        GW_ASSERT_EQ_INT(gw_canopen_hb_register(&hb, 2u, 500u), 1);
        GW_ASSERT_EQ_INT(gw_canopen_hb_register(&hb, 1u, 500u), GW_ERR_STATE);
        hb.nodes[0].last_seen_ms = t;
        hb.nodes[1].last_seen_ms = t;

        make_frame(&f, 0x701u, d, 1u);
        GW_ASSERT_EQ_INT(gw_canopen_hb_feed(&hb, &f, t), 0);
        GW_ASSERT_EQ_INT(hb.nodes[0].online, 1);
        GW_ASSERT_EQ_INT(hb.nodes[0].state, GW_NMT_STATE_OPERATIONAL);
        GW_ASSERT_EQ_INT(hb.online_count, 1u);

        /* 节点 2 也上线（用 COB-ID 0x702 心跳帧） */
        {
            gw_can_frame_t f2;
            make_frame(&f2, 0x702u, d, 1u);
            GW_ASSERT_EQ_INT(gw_canopen_hb_feed(&hb, &f2, t + 100u), 1);
        }
        GW_ASSERT_EQ_INT(hb.online_count, 2u);

        GW_ASSERT_EQ_INT(gw_canopen_hb_feed(&hb, &f, t + 400u), 0);   /* 未超时 */
        GW_ASSERT_EQ_INT(gw_canopen_hb_check(&hb, t + 450u), 0);   /* 节点2 也还没超时 */
        GW_ASSERT_EQ_INT(gw_canopen_hb_check(&hb, t + 950u), 2);      /* 两个都超时 */
        GW_ASSERT_EQ_INT(hb.online_count, 0u);
        GW_ASSERT_EQ_INT(hb.nodes[1].online, 0);
        GW_ASSERT_EQ_INT(hb.nodes[0].online, 0);
        GW_ASSERT_EQ_INT(hb.total_timeouts >= 2u, 1);

        /* 心跳恢复 -> 重新计入在线 */
        GW_ASSERT_EQ_INT(gw_canopen_hb_feed(&hb, &f, t + 1000u), 0);
        GW_ASSERT_EQ_INT(hb.nodes[0].online, 1);
        GW_ASSERT_EQ_INT(hb.online_count, 1u);

        /* 非心跳帧喂给心跳消费者应被拒绝 */
        make_frame(&f, 0x181u, d, 1u);
        GW_ASSERT_EQ_INT(gw_canopen_hb_feed(&hb, &f, t + 1010u), GW_ERR_UNSUPPORTED);
        /* 未注册的节点 */
        make_frame(&f, 0x709u, d, 1u);
        GW_ASSERT_EQ_INT(gw_canopen_hb_feed(&hb, &f, t + 1020u), GW_ERR_NOT_FOUND);
    }

    GW_CASE("对象字典：查找/读写/访问权限/类型换算");
    {
        const gw_od_entry_t *e;
        uint32_t v = 0u;

        e = gw_od_lookup(&s_od, 0x1017u, 0u);
        GW_ASSERT(e != NULL);
        GW_ASSERT_STR(e->name, "producer_hb_time");
        GW_ASSERT_EQ_INT(gw_od_read_u32(e, &v), GW_OK);
        GW_ASSERT_EQ_INT(v, 0u);
        GW_ASSERT_EQ_INT(gw_od_write_u32(e, 1000u), GW_OK);
        GW_ASSERT_EQ_INT(gw_od_read_u32(e, &v), GW_OK);
        GW_ASSERT_EQ_INT(v, 1000u);

        /* 只读对象写失败 */
        e = gw_od_lookup(&s_od, 0x1000u, 0u);
        GW_ASSERT_EQ_INT(gw_od_write_u32(e, 1u), GW_ERR_STATE);
        /* 有符号类型：写入负值能正确回读 */
        e = gw_od_lookup(&s_od, 0x2000u, 1u);
        GW_ASSERT_EQ_INT(gw_od_write_u32(e, (uint32_t)(int32_t)-1234), GW_OK);
        GW_ASSERT_EQ_INT(gw_od_read_u32(e, &v), GW_OK);
        GW_ASSERT_EQ_INT((int32_t)v, -1234);
        /* U8 截断 */
        e = gw_od_lookup(&s_od, 0x6000u, 1u);
        GW_ASSERT_EQ_INT(gw_od_write_u32(e, 0x1FFu), GW_OK);
        GW_ASSERT_EQ_INT(gw_od_read_u32(e, &v), GW_OK);
        GW_ASSERT_EQ_INT(v, 0xFFu);
        /* 不存在 */
        GW_ASSERT(gw_od_lookup(&s_od, 0x9999u, 0u) == NULL);
        GW_ASSERT_EQ_INT(gw_od_type_size(GW_OD_U32), 4u);
        GW_ASSERT_STR(gw_od_type_str(GW_OD_I16), "INT16");
    }

    GW_CASE("SDO 加速传输：下载请求编码 + 从站响应解析（4/2/1 字节）");
    {
        gw_can_frame_t f;
        gw_sdo_result_t r;

        GW_ASSERT_EQ_INT(gw_sdo_build_download_expedited(1u, 0x1017u, 0u, 1000u, 2u,
                                                         &f), GW_OK);
        GW_ASSERT_EQ_INT(f.id, 0x601u);
        GW_ASSERT_EQ_INT(f.data[0], GW_SDO_CS_DOWNLOAD_EXP_2B);
        GW_ASSERT_EQ_INT(f.data[1], 0x17u);       /* 索引低字节 */
        GW_ASSERT_EQ_INT(f.data[2], 0x10u);       /* 索引高字节 */
        GW_ASSERT_EQ_INT(f.data[3], 0x00u);
        GW_ASSERT_EQ_INT(f.data[4], 0xE8u);       /* 1000 = 0x03E8, 小端 */
        GW_ASSERT_EQ_INT(f.data[5], 0x03u);

        /* 从站确认帧 */
        {
            gw_can_frame_t ack;
            uint8_t d[8] = { GW_SDO_CS_DOWNLOAD_ACK, 0x17, 0x10, 0x00, 0, 0, 0, 0 };
            make_frame(&ack, 0x581u, d, 8u);
            GW_ASSERT_EQ_INT(gw_sdo_parse(&ack, &r), GW_OK);
            GW_ASSERT(r.valid);
            GW_ASSERT_EQ_INT(r.index, 0x1017u);
            GW_ASSERT_EQ_INT(r.size, 0u);
        }
        /* 上传响应的 4 种字节数 */
        {
            uint8_t d4[8] = { GW_SDO_CS_UPLOAD_EXP_4B, 0x00, 0x20, 0x01,
                              0x78, 0x56, 0x34, 0x12 };
            gw_can_frame_t up;
            make_frame(&up, 0x581u, d4, 8u);
            GW_ASSERT_EQ_INT(gw_sdo_parse(&up, &r), GW_OK);
            GW_ASSERT_EQ_INT(r.index, 0x2000u);
            GW_ASSERT_EQ_INT(r.subindex, 1u);
            GW_ASSERT_EQ_INT(r.size, 4u);
            GW_ASSERT_EQ_INT(r.value, 0x12345678u);
        }
        {
            uint8_t d1[8] = { GW_SDO_CS_UPLOAD_EXP_1B, 0x00, 0x60, 0x01,
                              0x5A, 0, 0, 0 };
            gw_can_frame_t up;
            make_frame(&up, 0x581u, d1, 8u);
            GW_ASSERT_EQ_INT(gw_sdo_parse(&up, &r), GW_OK);
            GW_ASSERT_EQ_INT(r.size, 1u);
            GW_ASSERT_EQ_INT(r.value, 0x5Au);
        }
    }

    GW_CASE("SDO 中止（Abort）解析出标准中止码");
    {
        uint8_t d[8] = { GW_SDO_CS_ABORT, 0x17, 0x10, 0x00,
                         0x02, 0x00, 0x00, 0x06 };   /* 0x06000002 无此对象 */
        gw_can_frame_t f;
        gw_sdo_result_t r;
        make_frame(&f, 0x581u, d, 8u);
        GW_ASSERT_EQ_INT(gw_sdo_parse(&f, &r), GW_ERR_EXCEPTION);
        GW_ASSERT(r.aborted);
        GW_ASSERT_EQ_INT(r.index, 0x1017u);
        GW_ASSERT_EQ_INT(r.abort_code, 0x06000002u);
    }

    GW_CASE("SDO 分段传输：初始化 + 2 段数据 + 段确认, 重组结果一致");
    {
        gw_can_frame_t  f;
        gw_sdo_result_t r;
        uint8_t         payload[10] = { 'G', 'A', 'T', 'E', 'W', 'A', 'Y', '0', '1', 'X' };
        uint8_t         rebuilt[16];
        uint32_t        rebuilt_len = 0u;
        bool            toggle = false;
        uint8_t         seg[8];
        uint8_t         seg_len = 0u;

        /* 1) 初始化下载：总长 10 字节 */
        GW_ASSERT_EQ_INT(gw_sdo_build_initiate_download(1u, 0x2000u, 2u, 10u, &f),
                         GW_OK);
        GW_ASSERT_EQ_INT(f.data[0], GW_SDO_CS_DOWNLOAD_INIT_SEG);
        GW_ASSERT_EQ_INT(f.data[4], 10u);
        GW_ASSERT_EQ_INT(f.data[5], 0u);

        /* 2) 第 1 段：7 字节, toggle=0 */
        GW_ASSERT_EQ_INT(gw_sdo_build_segment(1u, false, payload, 7u, &f), GW_OK);
        GW_ASSERT_EQ_INT(f.data[0], 0x00u);       /* n=0 -> 7 字节有效 */
        GW_ASSERT_EQ_INT(gw_sdo_parse_segment(&f, &toggle, seg, &seg_len), GW_OK);
        GW_ASSERT(!toggle);
        GW_ASSERT_EQ_INT(seg_len, 7u);
        memcpy(&rebuilt[rebuilt_len], seg, seg_len);
        rebuilt_len += seg_len;

        /* 从站段确认 */
        GW_ASSERT_EQ_INT(gw_sdo_build_segment_ack(1u, false, &f), GW_OK);
        GW_ASSERT_EQ_INT(f.data[0], GW_SDO_CS_DOWNLOAD_ACK);
        GW_ASSERT_EQ_INT(f.id, 0x581u);

        /* 3) 第 2 段：3 字节, toggle=1, n = 4 */
        GW_ASSERT_EQ_INT(gw_sdo_build_segment(1u, true, payload + 7u, 3u, &f), GW_OK);
        GW_ASSERT_EQ_INT(f.data[0], 0x14u);
        GW_ASSERT_EQ_INT(gw_sdo_parse_segment(&f, &toggle, seg, &seg_len), GW_OK);
        GW_ASSERT(toggle);
        GW_ASSERT_EQ_INT(seg_len, 3u);
        memcpy(&rebuilt[rebuilt_len], seg, seg_len);
        rebuilt_len += seg_len;

        GW_ASSERT_EQ_INT(rebuilt_len, 10u);
        GW_ASSERT_EQ_MEM(rebuilt, payload, 10u);

        /* 非法参数 */
        GW_ASSERT_EQ_INT(gw_sdo_build_segment(1u, false, payload, 8u, &f), GW_ERR_PARAM);
        GW_ASSERT_EQ_INT(gw_sdo_build_segment(1u, false, NULL, 1u, &f), GW_ERR_PARAM);
        GW_ASSERT_EQ_INT(gw_sdo_build_download_expedited(1u, 0x1000u, 0u, 1u, 5u,
                                                         &f), GW_ERR_PARAM);
        GW_UNUSED(r);
    }

    GW_CASE("PDO 映射：按映射表把 8 字节过程数据拆进对象字典");
    {
        gw_pdo_map_t map;
        uint8_t      data[8] = { 0x12, 0x34,        /* 电流 0x2000:1 = 0x1234 */
                                 0x00, 0x64,        /* 心跳时间 0x1017:0 = 100  */
                                 0x01,              /* DO CH1 = 1               */
                                 0x00, 0x00, 0x00 };
        int32_t v = 0;

        memset(&map, 0, sizeof(map));
        map.cob_id = 0x81u;
        map.count  = 3u;
        map.entries[0].index = 0x2000u; map.entries[0].subindex = 1u; map.entries[0].bits = 16u;
        map.entries[1].index = 0x1017u; map.entries[1].subindex = 0u; map.entries[1].bits = 16u;
        map.entries[2].index = 0x6000u; map.entries[2].subindex = 1u; map.entries[2].bits = 8u;

        GW_ASSERT_EQ_INT(gw_pdo_decode_to_od(&map, &s_od, data, 8u), GW_OK);
        GW_ASSERT_EQ_INT((int)s_od_i16, 0x1234);
        GW_ASSERT_EQ_INT(s_od_u16, 100u);
        GW_ASSERT_EQ_INT(s_od_u8, 1u);
        {
            const gw_od_entry_t *e = gw_od_lookup(&s_od, 0x2000u, 1u);
            GW_ASSERT_EQ_INT(gw_od_read_u32(e, (uint32_t *)&v), GW_OK);
            GW_ASSERT_EQ_INT(v, 0x1234);
        }

        /* 映射项指向不存在的对象 -> NOT_FOUND */
        map.entries[0].index = 0x7777u;
        GW_ASSERT_EQ_INT(gw_pdo_decode_to_od(&map, &s_od, data, 8u),
                         GW_ERR_NOT_FOUND);
        /* 数据长度不足 -> PARAM */
        map.entries[0].index = 0x2000u;
        GW_ASSERT_EQ_INT(gw_pdo_decode_to_od(&map, &s_od, data, 2u), GW_ERR_PARAM);
        /* 非 8/16/32 位宽 -> UNSUPPORTED */
        map.entries[0].bits = 24u;
        GW_ASSERT_EQ_INT(gw_pdo_decode_to_od(&map, &s_od, data, 8u),
                         GW_ERR_UNSUPPORTED);
    }

    GW_CASE("EMCY 紧急报文解析（错误码小端 + 错误寄存器 + 厂商数据）");
    {
        uint8_t d[8] = { 0x10, 0x23, 0x01, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE };
        gw_can_frame_t f;
        gw_emcy_t e;
        make_frame(&f, 0x083u, d, 8u);
        GW_ASSERT_EQ_INT(gw_canopen_emcy_parse(&f, &e), GW_OK);
        GW_ASSERT_EQ_INT(e.node_id, 3u);
        GW_ASSERT_EQ_INT(e.error_code, 0x2310u);
        GW_ASSERT_EQ_INT(e.error_register, 0x01u);
        GW_ASSERT_EQ_INT(e.vendor_data[0], 0xAAu);
        GW_ASSERT_EQ_INT(e.vendor_data[4], 0xEEu);

        make_frame(&f, 0x181u, d, 8u);
        GW_ASSERT_EQ_INT(gw_canopen_emcy_parse(&f, &e), GW_ERR_UNSUPPORTED);
    }
}
