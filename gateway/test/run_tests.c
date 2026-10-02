/*
 * run_tests.c - 单元测试总入口
 *
 * 每个套件独立执行, 最后汇总断言数与失败数；只要有一处断言失败,
 * 进程返回非 0, 可以直接挂到 CI（GitHub Actions / Jenkins）上。
 */
#include "test_util.h"
#include "port_rtos.h"
#include "gw_log.h"

#include <stdio.h>

int main(int argc, char **argv)
{
    bool verbose = false;
    int  i;

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--verbose") == 0) {
            verbose = true;
        }
    }

    gw_port_init();
    /* 默认只打印 WARN 以上, 避免测试输出被逐帧日志淹没；
     * 加 --verbose 可以看到优先级继承等过程的原始日志。 */
    gw_log_set_level(verbose ? GW_LOG_INFO : GW_LOG_ERROR);

    printf("========================================================\n");
    printf(" RT-Thread 工业通信网关 - 单元测试\n");
    printf(" 端口: %-14s 调度: %s\n", gw_port_name(), gw_port_sched_mode());
    printf("========================================================\n");

    test_crc_and_types();
    test_frame_sync();
    test_mem_pool();
    test_ring_buffer();
    test_modbus_rtu();
    test_canopen();
    test_bus_health();
    test_device_model();
    test_param_store();
    test_port_objects();
    test_priority_inherit();

    printf("\n========================================================\n");
    printf(" 测试用例组 : %d\n", g_case_total);
    printf(" 断言总数   : %d\n", g_assert_total);
    printf(" 失败断言   : %d\n", g_assert_failed);
    printf(" 结果       : %s\n", (g_assert_failed == 0) ? "PASS (0 失败)"
                                                        : "FAIL");
    printf("========================================================\n");

    return (g_assert_failed == 0) ? 0 : 1;
}
