/*
 * gw_host_sim.c - PC 端可执行程序（非目标板代码）
 *
 * 两个用途：
 *   1) --link loopback：进程内挂一个可注入故障的模拟从站, 完整跑一遍
 *      "正常轮询 -> CRC 错 -> 掉线 -> 恢复" 的端到端流程, 打印
 *      GW_STATS_JSON 供 tools/verify_protocol.py 解析；
 *   2) --link stdio / tcp-*：与 tools/sim_slave.py 对接, 由 Python 侧
 *      注入故障, 用于跨语言验证协议实现是否真的互操作。
 *
 * 场景脚本（--scenario mixed 时的默认序列）：
 *   前 normal1 次请求正常应答
 *   -> crc_bad 次故意破坏 CRC（模拟总线干扰）
 *   -> silent 次完全不应答（模拟设备掉电/断线）
 *   -> short_frm 次只回半个帧（模拟帧被截断）
 *   -> 之后恢复正常, 用于验证"恢复帧到达后重新纳入轮询"
 * Python 侧的 sim_slave.py 实现了完全相同的脚本, 便于交叉比对。
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "gw_main.h"
#include "gw_log.h"
#include "gw_crc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ================================================================== */
/* 命令行参数                                                          */
/* ================================================================== */
typedef struct {
    const char *link;        /* loopback | stdio | tcp-listen | tcp-connect */
    const char *addr;        /* tcp 地址 "127.0.0.1:15020" 或 "listen:15020" */
    const char *scenario;    /* normal | crc | offline | mixed              */
    const char *stats_out;   /* 统计输出文件（NULL = stdout）               */
    const char *report_out;  /* 上报输出文件（NULL = stdout）               */
    uint32_t    duration_ms;
    uint32_t    polls;
    bool        verbose;
    bool        tcp_mode;    /* --mode tcp 时链路跑 Modbus TCP 而非 RTU     */
} sim_args_t;

static void usage(void)
{
    fprintf(stderr,
        "usage: gw_host_sim [options]\n"
        "  --link <loopback|stdio|tcp-listen|tcp-connect>   default loopback\n"
        "  --addr <ip:port|listen:port>                     tcp link address\n"
        "  --mode <rtu|tcp>                                 application protocol\n"
        "  --scenario <normal|crc|offline|mixed>            fault injection script\n"
        "  --duration <ms>                                  run time, default 2500\n"
        "  --polls <n>                                      stop after n polls\n"
        "  --stats-out <file>                               write GW_STATS_JSON here\n"
        "  --report-out <file>                              write report JSON here\n"
        "  --verbose                                        enable DEBUG logs\n");
}

/* ================================================================== */
/* 进程内模拟从站 + 故障注入                                           */
/* ================================================================== */
#define SIM_MAX_SLAVES 4u

/* 每台"模拟设备"独立维护一条故障脚本时间线：
 *   前 normal1 次请求正常应答 -> 注入 script_crc 次 CRC 错
 *   -> 注入 script_silent 次静默 -> 注入 script_short 次截断帧 -> 之后永久正常
 * 这样每台设备都能积累出"连续 N 次同类错误"以触发隔离, 也都能在
 * 故障脚本走完后产生恢复帧（用于验证重新纳入轮询）。 */
typedef struct {
    gw_mb_slave_t slave;
    uint32_t      req_index;
    uint32_t      crc_injected;
    uint32_t      silent_injected;
    uint32_t      short_injected;
    uint32_t      ok_responses;
    uint32_t      script_crc;
    uint32_t      script_silent;
    uint32_t      script_short;
} sim_dev_t;

typedef struct {
    sim_dev_t dev[SIM_MAX_SLAVES];
    uint32_t  slave_count;
    uint32_t  normal1;
    uint32_t  total_req;
    uint32_t  total_crc;
    uint32_t  total_silent;
    uint32_t  total_short;
    uint32_t  total_ok;
} sim_slave_ctx_t;

static void sim_script_init(sim_slave_ctx_t *c, const char *scenario)
{
    memset(c, 0, sizeof(*c));

    if ((scenario == NULL) || (strcmp(scenario, "mixed") == 0)) {
        /* dev0(焊机, addr 1)：连续 CRC 错 -> 验证 CRC 类隔离
         * dev1(机器人, addr 2)：连续静默 -> 验证掉线类隔离
         * 两台都会在脚本走完后恢复正常, 用于验证自动恢复 */
        c->normal1          = 12u;
        c->dev[0].script_crc    = 6u;
        c->dev[1].script_silent = 8u;
    } else if (strcmp(scenario, "crc") == 0) {
        c->normal1 = 10u;
        c->dev[0].script_crc = 6u;
        c->dev[1].script_crc = 6u;
    } else if (strcmp(scenario, "offline") == 0) {
        c->normal1 = 10u;
        c->dev[0].script_silent = 8u;
        c->dev[1].script_silent = 8u;
    } else if (strcmp(scenario, "short") == 0) {
        c->normal1 = 10u;
        c->dev[0].script_short = 6u;
        c->dev[1].script_short = 6u;
    } else {                     /* normal：永不注入 */
        c->normal1 = 0xFFFFFFFFu;
    }
}

static int sim_slave_fn(void *ctx, const uint8_t *req, size_t req_len,
                        uint8_t *resp, size_t resp_cap)
{
    sim_slave_ctx_t *c = (sim_slave_ctx_t *)ctx;
    sim_dev_t       *d;
    uint32_t         i;
    uint32_t         n = 0u;

    if ((c == NULL) || (req == NULL) || (req_len < 4u)) {
        return 0;
    }

    for (i = 0u; i < c->slave_count; i++) {
        if (c->dev[i].slave.addr == req[0]) {
            break;
        }
    }
    if (i >= c->slave_count) {
        return 0;                             /* 地址不匹配：静默 */
    }
    d = &c->dev[i];
    d->req_index++;
    c->total_req++;

    /* --- 脚本时间线：正常 -> CRC 错 -> 静默 -> 截断 -> 正常 --- */
    if (d->req_index > c->normal1) {
        if (d->crc_injected < d->script_crc) {
            uint32_t rc = (uint32_t)gw_mb_slave_process(&d->slave, req,
                                                        (uint32_t)req_len, resp,
                                                        (uint32_t)resp_cap, &n);
            if (((rc == GW_OK) || (rc == (uint32_t)GW_ERR_EXCEPTION)) && (n >= 2u)) {
                d->crc_injected++;
                c->total_crc++;
                d->ok_responses++;
                resp[n - 1u] ^= 0xFFu;        /* 翻转 CRC 高字节 */
                return (int)n;
            }
            return 0;
        }
        if (d->silent_injected < d->script_silent) {
            d->silent_injected++;
            c->total_silent++;
            return 0;                         /* 完全不应答 */
        }
        if (d->short_injected < d->script_short) {
            uint32_t rc = (uint32_t)gw_mb_slave_process(&d->slave, req,
                                                        (uint32_t)req_len, resp,
                                                        (uint32_t)resp_cap, &n);
            if (((rc == GW_OK) || (rc == (uint32_t)GW_ERR_EXCEPTION)) && (n > 4u)) {
                d->short_injected++;
                c->total_short++;
                d->ok_responses++;
                return (int)(n - 3u);         /* 少发 3 字节: 典型截断帧 */
            }
            return 0;
        }
    }

    {
        uint32_t rc = (uint32_t)gw_mb_slave_process(&d->slave, req,
                                                    (uint32_t)req_len, resp,
                                                    (uint32_t)resp_cap, &n);
        if ((rc != GW_OK) && (rc != (uint32_t)GW_ERR_EXCEPTION)) {
            return 0;
        }
    }
    d->ok_responses++;
    c->total_ok++;
    return (int)n;
}

/* ================================================================== */
/* 上报出口                                                            */
/* ================================================================== */
static FILE *s_stats_fp  = NULL;
static FILE *s_report_fp = NULL;
static uint32_t s_report_count = 0u;

static int report_sink(void *ctx, const gw_report_t *r)
{
    (void)ctx;
    if (s_report_fp == NULL) {
        return GW_OK;
    }
    s_report_count++;
    fprintf(s_report_fp,
            "GW_REPORT_JSON {\"dev\":%u,\"param\":%u,\"value\":%d,\"ts\":%u}\n",
            (unsigned)r->dev_id, (unsigned)r->param_id, (int)r->value,
            (unsigned)r->ts_ms);
    return GW_OK;
}

static void stats_sink(void *ctx, const char *line)
{
    FILE *fp = (FILE *)ctx;
    if (fp != NULL) {
        fputs(line, fp);
        fflush(fp);
    }
}

/* ================================================================== */
/* main                                                               */
/* ================================================================== */
static gw_gateway_t s_gateway;    /* 静态分配：启动期一次性占用, 无堆分配 */

int main(int argc, char **argv)
{
    sim_args_t     args;
    sim_slave_ctx_t sim;
    gw_link_t     *link = NULL;
    gw_gateway_cfg_t cfg;
    uint32_t       start_ms;
    int            i;

    memset(&args, 0, sizeof(args));
    args.link        = "loopback";
    args.scenario    = "mixed";
    args.duration_ms = 2500u;
    args.polls       = 0u;

    for (i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "--link") == 0) && (i + 1 < argc)) {
            args.link = argv[++i];
        } else if ((strcmp(argv[i], "--addr") == 0) && (i + 1 < argc)) {
            args.addr = argv[++i];
        } else if ((strcmp(argv[i], "--mode") == 0) && (i + 1 < argc)) {
            args.tcp_mode = (strcmp(argv[++i], "tcp") == 0);
        } else if ((strcmp(argv[i], "--scenario") == 0) && (i + 1 < argc)) {
            args.scenario = argv[++i];
        } else if ((strcmp(argv[i], "--duration") == 0) && (i + 1 < argc)) {
            args.duration_ms = (uint32_t)strtoul(argv[++i], NULL, 10);
        } else if ((strcmp(argv[i], "--polls") == 0) && (i + 1 < argc)) {
            args.polls = (uint32_t)strtoul(argv[++i], NULL, 10);
        } else if ((strcmp(argv[i], "--stats-out") == 0) && (i + 1 < argc)) {
            args.stats_out = argv[++i];
        } else if ((strcmp(argv[i], "--report-out") == 0) && (i + 1 < argc)) {
            args.report_out = argv[++i];
        } else if (strcmp(argv[i], "--verbose") == 0) {
            args.verbose = true;
        } else if ((strcmp(argv[i], "--help") == 0) || (strcmp(argv[i], "-h") == 0)) {
            usage();
            return 0;
        } else {
            fprintf(stderr, "unknown option: %s\n", argv[i]);
            usage();
            return 2;
        }
    }

    gw_port_init();
    gw_log_set_level(args.verbose ? GW_LOG_DEBUG : GW_LOG_INFO);

    s_stats_fp  = stdout;
    s_report_fp = stdout;
    if (args.stats_out != NULL) {
        s_stats_fp = fopen(args.stats_out, "w");
        if (s_stats_fp == NULL) {
            fprintf(stderr, "cannot open stats file %s\n", args.stats_out);
            return 3;
        }
    }
    if (args.report_out != NULL) {
        s_report_fp = fopen(args.report_out, "w");
        if (s_report_fp == NULL) {
            fprintf(stderr, "cannot open report file %s\n", args.report_out);
            return 3;
        }
    }

    /* ---- 选择链路 ---- */
    if (strcmp(args.link, "loopback") == 0) {
        sim_script_init(&sim, args.scenario);
        (void)gw_mb_slave_init(&sim.dev[0].slave, 1u, 256u, 32u);
        (void)gw_mb_slave_init(&sim.dev[1].slave, 2u, 256u, 32u);
        sim.slave_count = 2u;
        /* 给模拟从站填一些可信的初始数据（焊接电流 180.0A 等） */
        sim.dev[0].slave.holding[0x0000] = 1800u;
        sim.dev[0].slave.holding[0x0001] = 240u;
        sim.dev[0].slave.holding[0x0002] = 850u;
        sim.dev[0].slave.holding[0x0003] = 1782u;
        sim.dev[0].slave.holding[0x0004] = 238u;
        sim.dev[0].slave.holding[0x0005] = 2u;
        sim.dev[0].slave.holding[0x0009] = 152u;
        sim.dev[0].slave.holding[0x000A] = 1u;
        sim.dev[0].slave.holding[0x000B] = 3u;
        sim.dev[1].slave.holding[0x0000] = 1u;
        sim.dev[1].slave.holding[0x0002] = (uint16_t)(int16_t)(-1250);
        sim.dev[1].slave.holding[0x0003] = 4500u;
        sim.dev[1].slave.holding[0x000E] = 0u;
        sim.dev[1].slave.holding[0x0011] = 1u;
        link = gw_link_loopback_create(sim_slave_fn, &sim);
    } else if (strcmp(args.link, "stdio") == 0) {
        link = gw_link_stdio_create();
    } else if (strcmp(args.link, "tcp-listen") == 0) {
        link = gw_link_tcp_listen_create();
    } else if (strcmp(args.link, "tcp-connect") == 0) {
        link = gw_link_tcp_connect_create();
    } else {
        fprintf(stderr, "unknown link: %s\n", args.link);
        return 2;
    }
    if (link == NULL) {
        fprintf(stderr, "link create failed\n");
        return 3;
    }
    if (gw_link_open(link, args.addr) != GW_OK) {
        fprintf(stderr, "link open failed (%s / %s)\n", args.link,
                (args.addr != NULL) ? args.addr : "-");
        return 3;
    }

    /* stdio 模式下协议字节占用 stdout, 统计必须走文件 */
    if ((strcmp(args.link, "stdio") == 0) && (args.stats_out == NULL)) {
        fprintf(stderr, "--link stdio requires --stats-out <file>\n");
        return 2;
    }

    /* ---- 组装网关 ---- */
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode            = args.tcp_mode ? GW_LINK_MODE_TCP : GW_LINK_MODE_RTU;
    cfg.resp_timeout_ms = GW_MB_RESP_TIMEOUT_MS;
    cfg.retries         = GW_MB_RETRY_MAX;
    cfg.poll_period_ms  = GW_POLL_PERIOD_MS;
    cfg.verbose         = args.verbose;

    if (gw_gateway_init(&s_gateway, link, &cfg) != GW_OK) {
        fprintf(stderr, "gateway init failed\n");
        return 4;
    }
    (void)gw_gateway_register_device(&s_gateway, 1u, GW_PROTO_MODBUS_RTU,
                                     GW_PROFILE_WELDER, "digital-welder");
    (void)gw_gateway_register_device(&s_gateway, 2u, GW_PROTO_MODBUS_RTU,
                                     GW_PROFILE_ROBOT, "six-axis-robot");
    if (args.tcp_mode) {
        (void)gw_gateway_register_device(&s_gateway, 3u, GW_PROTO_MODBUS_TCP,
                                         GW_PROFILE_PLC, "plc-remote-io");
    }
    gw_gateway_set_report_sink(&s_gateway, report_sink, NULL);
    gw_gateway_set_stats_sink(&s_gateway, stats_sink, s_stats_fp);

    if (gw_gateway_start(&s_gateway) != GW_OK) {
        fprintf(stderr, "gateway start failed\n");
        return 4;
    }

    /* ---- 跑指定时长或指定轮询次数 ---- */
    start_ms = gw_port_tick_ms();
    while ((gw_port_tick_ms() - start_ms) < args.duration_ms) {
        gw_port_delay_ms(20);
        if ((args.polls > 0u) && (s_gateway.stats.polls_attempted >= args.polls)) {
            break;
        }
    }

    gw_gateway_stop(&s_gateway);

    /* ---- 输出结构化统计 ---- */
    gw_gateway_dump_stats(&s_gateway);
    GW_LOGI("SIM", "report lines emitted: %u", (unsigned)s_report_count);

    GW_LOGI("SIM", "simulated slave side: requests=%u ok_resp=%u "
                   "crc_injected=%u silent=%u short=%u",
            (unsigned)sim.total_req, (unsigned)(sim.total_ok + sim.total_crc + sim.total_short),
            (unsigned)sim.total_crc, (unsigned)sim.total_silent,
            (unsigned)sim.total_short);

    if ((s_stats_fp != NULL) && (s_stats_fp != stdout)) {
        fclose(s_stats_fp);
    }
    if ((s_report_fp != NULL) && (s_report_fp != stdout)) {
        fclose(s_report_fp);
    }
    gw_link_close(link);
    return 0;
}
