/*
 * test_common.c - 公共断言计数器 + CRC/基础类型测试
 */
#include "test_util.h"
#include "gw_crc.h"
#include "gw_types.h"

#include <stdlib.h>

int g_assert_total = 0;
int g_assert_failed = 0;
int g_case_total = 0;

void test_crc_and_types(void)
{
    static const uint8_t vec[] = "123456789";
    uint8_t frame[16];
    uint16_t c1;
    uint16_t c2;
    int i;

    GW_SUITE("CRC16 / 基础类型");

    GW_CASE("标准校验向量 \"123456789\" -> 0x4B37");
    c1 = gw_crc16_modbus(vec, 9u);
    GW_ASSERT_EQ_INT(c1, 0x4B37);
    c2 = gw_crc16_modbus_fast(vec, 9u);
    GW_ASSERT_EQ_INT(c2, 0x4B37);          /* 查表实现必须与逐位实现一致 */

    GW_CASE("空数据初值应为 0xFFFF");
    GW_ASSERT_EQ_INT(gw_crc16_modbus(NULL, 0u), 0xFFFF);

    GW_CASE("分片累加 == 一次性计算");
    c1 = gw_crc16_modbus_update(0xFFFFu, vec, 4u);
    c1 = gw_crc16_modbus_update(c1, vec + 4, 5u);
    GW_ASSERT_EQ_INT(c1, 0x4B37);

    GW_CASE("查表与逐位实现在 4096 组随机数据上完全一致");
    {
        uint8_t buf[37];
        int ok = 1;
        srand(20240513);
        for (i = 0; i < 4096; i++) {
            int n = 1 + (rand() % (int)sizeof(buf));
            int k;
            for (k = 0; k < n; k++) {
                buf[k] = (uint8_t)(rand() & 0xFF);
            }
            if (gw_crc16_modbus(buf, (size_t)n) !=
                gw_crc16_modbus_fast(buf, (size_t)n)) {
                ok = 0;
                break;
            }
        }
        GW_ASSERT(ok);
    }

    GW_CASE("帧尾 CRC 追加与校验（低字节在前）");
    frame[0] = 0x01; frame[1] = 0x03;
    frame[2] = 0x00; frame[3] = 0x00;
    frame[4] = 0x00; frame[5] = 0x0A;
    gw_crc16_modbus_append(frame, 6u);
    /* 已知向量：01 03 00 00 00 0A -> CRC 值 0xCDC5, 线上低字节在前 = C5 CD */
    GW_ASSERT_EQ_INT(frame[6], 0xC5);
    GW_ASSERT_EQ_INT(frame[7], 0xCD);
    GW_ASSERT(gw_crc16_modbus_check_frame(frame, 8u));
    frame[3] ^= 0x01;
    GW_ASSERT(!gw_crc16_modbus_check_frame(frame, 8u));

    GW_CASE("CRC8 (poly 0x07) 已知向量");
    {
        static const uint8_t v2[] = { 0x01, 0x02, 0x03 };
        uint8_t c = gw_crc8(v2, 3u);
        GW_ASSERT_EQ_INT(c, 0x48);
    }

    GW_CASE("大端读写与枚举转字符串");
    {
        uint8_t b[4];
        gw_write_be32(b, 0x12345678u);
        GW_ASSERT_EQ_INT(b[0], 0x12);
        GW_ASSERT_EQ_INT(b[3], 0x78);
        GW_ASSERT_EQ_INT(gw_read_be32(b), 0x12345678u);
        gw_write_be16(b, 0xABCDu);
        GW_ASSERT_EQ_INT(gw_read_be16(b), 0xABCDu);
        GW_ASSERT_STR(gw_status_str(GW_ERR_CRC), "ERR_CRC");
        GW_ASSERT_STR(gw_bus_state_str(GW_BUS_ISOLATED), "ISOLATED");
        GW_ASSERT_STR(gw_protocol_str(GW_PROTO_CANOPEN), "CANOPEN");
        GW_ASSERT_STR(gw_dev_state_str(GW_DEV_OFFLINE), "OFFLINE");
    }
}
