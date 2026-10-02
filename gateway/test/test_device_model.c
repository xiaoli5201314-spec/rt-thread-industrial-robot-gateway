/*
 * test_device_model.c - 数据模型层与参数持久化测试
 */
#include "test_util.h"
#include "device_model.h"
#include "param_store.h"
#include "port_hw.h"

#include <string.h>

void test_device_model(void)
{
    gw_devm_t m;
    int       d1;
    int       d2;
    uint32_t  now = 1000u;

    GW_SUITE("数据模型层（设备注册 / 心跳 / 参数映射 / 缓存队列）");

    GW_CASE("初始化与三个内建设备档案");
    GW_ASSERT_EQ_INT(gw_devm_init(&m), GW_OK);
    GW_ASSERT(gw_devm_profile(GW_PROFILE_WELDER) != NULL);
    GW_ASSERT(gw_devm_profile(GW_PROFILE_ROBOT) != NULL);
    GW_ASSERT(gw_devm_profile(GW_PROFILE_PLC) != NULL);
    GW_ASSERT_EQ_INT(gw_devm_profile(GW_PROFILE_WELDER)->field_count, 12u);
    GW_ASSERT_STR(gw_devm_profile(GW_PROFILE_WELDER)->name, "digital-welder");

    GW_CASE("注册设备：返回自增 ID, 重复地址被拒绝");
    d1 = gw_devm_register(&m, 1u, GW_PROTO_MODBUS_RTU, GW_PROFILE_WELDER,
                          "welder-1", now);
    GW_ASSERT_EQ_INT(d1, 1);
    d2 = gw_devm_register(&m, 2u, GW_PROTO_MODBUS_RTU, GW_PROFILE_ROBOT,
                          "robot-1", now);
    GW_ASSERT_EQ_INT(d2, 2);
    GW_ASSERT_EQ_INT(gw_devm_register(&m, 1u, GW_PROTO_MODBUS_RTU,
                                      GW_PROFILE_WELDER, "dup", now),
                     GW_ERR_STATE);
    GW_ASSERT_EQ_INT(m.device_count, 2u);
    GW_ASSERT(gw_devm_find(&m, 1) != NULL);
    GW_ASSERT(gw_devm_find_by_addr(&m, 2u, GW_PROTO_MODBUS_RTU) != NULL);
    GW_ASSERT(gw_devm_find_by_addr(&m, 2u, GW_PROTO_MODBUS_TCP) == NULL);
    GW_ASSERT(gw_devm_find(&m, 99) == NULL);

    GW_CASE("心跳：收到即 ONLINE；超时后转 OFFLINE 并计数");
    GW_ASSERT_EQ_INT(gw_devm_heartbeat(&m, 1u, now), GW_OK);
    GW_ASSERT_EQ_INT(gw_devm_find(&m, 1)->state, (int)GW_DEV_ONLINE);
    GW_ASSERT_EQ_INT(gw_devm_find(&m, 1)->online_count, 1u);
    now += GW_HEARTBEAT_TIMEOUT_MS + 10u;
    GW_ASSERT_EQ_INT(gw_devm_check_timeouts(&m, now), 1);   /* 设备 2 从未心跳 */
    GW_ASSERT_EQ_INT(gw_devm_find(&m, 1)->state, (int)GW_DEV_OFFLINE);
    GW_ASSERT_EQ_INT(gw_devm_find(&m, 1)->offline_count, 1u);
    GW_ASSERT_EQ_INT(gw_devm_check_timeouts(&m, now + 10u), 0);  /* 不重复计数 */

    GW_CASE("轮询结果驱动状态迁移（OK->ONLINE, 连续失败->DEGRADED）");
    gw_devm_report_poll_ok(&m, gw_devm_find(&m, 1), now);
    GW_ASSERT_EQ_INT(gw_devm_find(&m, 1)->state, (int)GW_DEV_ONLINE);
    gw_devm_report_poll_fail(&m, gw_devm_find(&m, 1), GW_ERRCLASS_TIMEOUT, now);
    gw_devm_report_poll_fail(&m, gw_devm_find(&m, 1), GW_ERRCLASS_TIMEOUT, now);
    GW_ASSERT_EQ_INT(gw_devm_find(&m, 1)->state, (int)GW_DEV_DEGRADED);
    GW_ASSERT_EQ_INT(gw_devm_find(&m, 1)->timeouts, 2u);

    GW_CASE("apply_read + 参数在线映射：原始值 -> 工程值（整数比例换算）");
    {
        /* 焊机保持寄存器：电流设定 0x0000(=1800, 0.1A) 电压 0x0001(=240) */
        uint16_t regs[13];
        memset(regs, 0, sizeof(regs));
        regs[0x0000] = 1800u;      /* 180.0 A */
        regs[0x0001] = 240u;       /* 24.0 V  */
        regs[0x0002] = 850u;       /* 8.50 m/min */
        regs[0x0005] = 2u;         /* 焊接中 */
        regs[0x0007] = 0x0001u;    /* 累计时长 U32 高字 */
        regs[0x0008] = 0x86A0u;    /* = 34464 -> 总 100000 s */
        regs[0x000C] = (uint16_t)(int16_t)(-25);  /* 弧力 -2.5 */
        GW_ASSERT_EQ_INT(gw_devm_apply_read(&m, gw_devm_find(&m, 1), 0u, regs,
                                            13u, now), GW_OK);
        {
            int32_t v = 0;
            GW_ASSERT_EQ_INT(gw_devm_read_param(&m, 1u, 0x1001u, &v), GW_OK);
            GW_ASSERT_EQ_INT(v, 180);           /* 1800 / 10 */
            GW_ASSERT_EQ_INT(gw_devm_read_param(&m, 1u, 0x1002u, &v), GW_OK);
            GW_ASSERT_EQ_INT(v, 24);
            GW_ASSERT_EQ_INT(gw_devm_read_param(&m, 1u, 0x1003u, &v), GW_OK);
            GW_ASSERT_EQ_INT(v, 8);             /* 850 / 100 */
            GW_ASSERT_EQ_INT(gw_devm_read_param(&m, 1u, 0x1006u, &v), GW_OK);
            GW_ASSERT_EQ_INT(v, 2);
            GW_ASSERT_EQ_INT(gw_devm_read_param(&m, 1u, 0x1008u, &v), GW_OK);
            GW_ASSERT_EQ_INT(v, 100000);        /* U32 双寄存器, 高字在前 */
            GW_ASSERT_EQ_INT(gw_devm_read_param(&m, 1u, 0x100Cu, &v), GW_OK);
            GW_ASSERT_EQ_INT(v, -2);            /* 有符号: -25/10 = -2 (整除) */
        }
        /* 不在本次轮询窗口内的参数返回 NOT_FOUND */
        {
            int32_t v = 0;
            GW_ASSERT_EQ_INT(gw_devm_read_param(&m, 2u, 0x2001u, &v),
                             GW_ERR_NOT_READY);
            GW_ASSERT_EQ_INT(gw_devm_read_param(&m, 1u, 0x9999u, &v),
                             GW_ERR_NOT_FOUND);
        }
    }

    GW_CASE("参数写缓存队列：离线写入进队列, 后写覆盖前写, 回放后为空");
    {
        gw_param_cache_item_t item;
        GW_ASSERT_EQ_INT(gw_devm_write_param(&m, 1u, 0x1001u, 200, now + 1u),
                         GW_OK);       /* 设备 DEGRADED -> 进缓存 */
        GW_ASSERT_EQ_INT(gw_devm_cache_count(&m), 1u);
        GW_ASSERT_EQ_INT(gw_devm_write_param(&m, 1u, 0x1001u, 210, now + 2u),
                         GW_OK);       /* 同一寄存器: 覆盖 */
        GW_ASSERT_EQ_INT(gw_devm_cache_count(&m), 1u);
        GW_ASSERT_EQ_INT(gw_devm_write_param(&m, 1u, 0x1002u, 25, now + 3u), GW_OK);
        GW_ASSERT_EQ_INT(gw_devm_cache_count(&m), 2u);

        GW_ASSERT_EQ_INT(gw_devm_cache_pop(&m, &item), GW_OK);
        GW_ASSERT_EQ_INT(item.dev_id, 1u);
        GW_ASSERT_EQ_INT(item.reg, 0x0000u);
        GW_ASSERT_EQ_INT(item.value, 2100u);      /* 210 * 10 -> 原始值 */
        GW_ASSERT_EQ_INT(gw_devm_cache_count(&m), 1u);
        GW_ASSERT_EQ_INT(gw_devm_cache_pop(&m, &item), GW_OK);
        GW_ASSERT_EQ_INT(item.value, 250u);
        GW_ASSERT_EQ_INT(gw_devm_cache_pop(&m, &item), GW_ERR_EMPTY);
        GW_ASSERT_EQ_INT(m.cache_flushed, 2u);
    }

    GW_CASE("缓存队列满时丢弃并计数（不会无限增长吃光内存）");
    {
        uint32_t i;
        GW_ASSERT_EQ_INT(gw_devm_cache_release(&m, 1u) >= 0, 1);
        for (i = 0u; i < GW_PARAM_CACHE_DEPTH; i++) {
            GW_ASSERT_EQ_INT(gw_devm_cache_push(&m, 1u, (uint16_t)(0x40u + i),
                                                (uint16_t)i, now), GW_OK);
        }
        GW_ASSERT_EQ_INT(gw_devm_cache_push(&m, 1u, 0x7FFu, 1u, now), GW_ERR_FULL);
        GW_ASSERT_EQ_INT(m.cache_drop, 1u);
        GW_ASSERT_EQ_INT(gw_devm_cache_release(&m, 1u), (int)GW_PARAM_CACHE_DEPTH);
        GW_ASSERT_EQ_INT(gw_devm_cache_count(&m), 0u);
    }

    GW_CASE("机器人档案：S16 关节角度与 S32 坐标换算");
    {
        uint16_t regs[18];
        int32_t  v = 0;
        memset(regs, 0, sizeof(regs));
        regs[0x0002] = (uint16_t)(int16_t)(-1250);       /* 关节 1: -12.50 deg */
        regs[0x0008] = 0x0000u;                          /* TCP X 高字 */
        regs[0x0009] = 0x2710u;                          /* 10000 -> 100.00 mm */
        GW_ASSERT_EQ_INT(gw_devm_apply_read(&m, gw_devm_find(&m, 2), 0u, regs,
                                            18u, now), GW_OK);
        GW_ASSERT_EQ_INT(gw_devm_read_param(&m, 2u, 0x2003u, &v), GW_OK);
        GW_ASSERT_EQ_INT(v, -12);
        GW_ASSERT_EQ_INT(gw_devm_read_param(&m, 2u, 0x2009u, &v), GW_OK);
        GW_ASSERT_EQ_INT(v, 100);
    }
}

void test_param_store(void)
{
    gw_param_store_t ps;
    gw_param_blob_t  out;
    gw_param_blob_t  in;
    uint16_t         len = 0u;

    GW_SUITE("掉电参数保存（双槽轮换 + 校验回读 + 上电自恢复）");

    GW_CASE("默认参数与首次上电（无有效镜像 -> 用出厂值落盘）");
    gw_param_defaults(&in);
    GW_ASSERT_EQ_INT(in.magic, GW_PARAM_MAGIC);
    GW_ASSERT_EQ_INT(in.version, 1u);
    GW_ASSERT_EQ_INT(gw_param_store_init(&ps), GW_OK);
    GW_ASSERT_EQ_INT(gw_param_load(&ps, &out, sizeof(out), &len), GW_ERR_NOT_FOUND);

    GW_CASE("保存 -> 回读一致（槽 0 -> 槽 1 轮换）");
    in.welder_current_set = 1900u;
    GW_ASSERT_EQ_INT(gw_param_save(&ps, &in, (uint16_t)sizeof(in)), GW_OK);
    GW_ASSERT_EQ_INT(ps.active_slot, 0u);
    GW_ASSERT_EQ_INT(gw_param_load(&ps, &out, sizeof(out), &len), GW_OK);
    GW_ASSERT_EQ_INT(len, (int)sizeof(in));
    GW_ASSERT_EQ_MEM(&out, &in, sizeof(in));
    GW_ASSERT_EQ_INT(out.welder_current_set, 1900u);

    in.welder_current_set = 2000u;
    GW_ASSERT_EQ_INT(gw_param_save(&ps, &in, (uint16_t)sizeof(in)), GW_OK);
    GW_ASSERT_EQ_INT(ps.active_slot, 1u);          /* 轮换到槽 1 */
    GW_ASSERT_EQ_INT(gw_param_load(&ps, &out, sizeof(out), &len), GW_OK);
    GW_ASSERT_EQ_INT(out.welder_current_set, 2000u);
    GW_ASSERT_EQ_INT(ps.save_count, 2u);

    GW_CASE("擦写次数被统计（说明轮换可降低单页磨损）");
    GW_ASSERT(gw_nv_get_erase_count() >= 2u);
    GW_ASSERT(gw_nv_get_write_count() >= 2u);

    GW_CASE("写一半掉电：新槽作废, 旧槽数据完好, 上电仍能读回旧值");
    {
        gw_param_store_t ps2;
        gw_param_blob_t  blob;
        uint16_t         l2 = 0u;
        uint32_t         old_value;

        GW_ASSERT_EQ_INT(gw_param_store_init(&ps2), GW_OK);
        gw_param_defaults(&blob);
        blob.robot_program_no = 7u;
        GW_ASSERT_EQ_INT(gw_param_save(&ps2, &blob, (uint16_t)sizeof(blob)), GW_OK);
        old_value = blob.robot_program_no;

        blob.robot_program_no = 9u;
        gw_nv_simulate_torn_write(true);
        GW_ASSERT_EQ_INT(gw_param_save(&ps2, &blob, (uint16_t)sizeof(blob)),
                         GW_ERR_IO);
        gw_nv_simulate_torn_write(false);
        GW_ASSERT(ps2.corrupt_count >= 1u);
        /* 掉电重启后重新加载：必须拿到旧值 7, 而不是半截数据 */
        GW_ASSERT_EQ_INT(gw_param_store_init(&ps2), GW_OK);
        GW_ASSERT_EQ_INT(gw_param_load(&ps2, &blob, sizeof(blob), &l2), GW_OK);
        GW_ASSERT_EQ_INT(blob.robot_program_no, old_value);
        GW_ASSERT_EQ_INT(ps2.recovery_count, 1u);      /* 主槽坏, 用备份槽救回 */

        /* 恢复供电后再保存一次, 系统回到正常双槽状态 */
        blob.robot_program_no = 11u;
        GW_ASSERT_EQ_INT(gw_param_save(&ps2, &blob, (uint16_t)sizeof(blob)), GW_OK);
        GW_ASSERT_EQ_INT(gw_param_load(&ps2, &blob, sizeof(blob), &l2), GW_OK);
        GW_ASSERT_EQ_INT(blob.robot_program_no, 11u);
    }

    GW_CASE("恢复出厂设置：写入默认值并可读回");
    GW_ASSERT_EQ_INT(gw_param_factory_reset(&ps), GW_OK);
    GW_ASSERT_EQ_INT(gw_param_load(&ps, &out, sizeof(out), &len), GW_OK);
    GW_ASSERT_EQ_INT(out.welder_current_set, 1800u);
    GW_ASSERT_EQ_INT(out.wire_feed_speed, 850u);
    GW_ASSERT_EQ_INT(ps.factory_reset_count, 1u);

    GW_CASE("参数长度异常被拒绝（防止把非法长度写进 Flash）");
    GW_ASSERT_EQ_INT(gw_param_save(&ps, &in, 0u), GW_ERR_PARAM);
    GW_ASSERT_EQ_INT(gw_param_save(&ps, &in, GW_PARAM_MAX_PAYLOAD + 1u), GW_ERR_PARAM);
    GW_ASSERT_EQ_INT(gw_param_save(&ps, NULL, 8u), GW_ERR_PARAM);
}
