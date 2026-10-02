# ============================================================================
# RT-Thread 工业通信网关 - 网关模块构建（在 gateway/ 目录下执行）
#
# 目标板固件请用 RT-Thread 的 scons 构建（见 gateway/SConscript）；
# 本 Makefile 只负责在 Linux/WSL/Windows-MinGW 上把同一份源码编译成
# "可在 PC 上真实运行的单元测试与整机仿真程序"，让协议栈、状态机、
# 内存池这些逻辑真正被执行与断言，而不是写完就算。
#
# 常用目标（也可以在仓库根目录用同名目标）：
#   make            编译 build/gw_host_sim（整机仿真程序）
#   make test       编译并运行全部单元测试（0 失败才算通过）
#   make verify     跑 tools/verify_protocol.py 端到端协议验证
#   make clean      清理
# ============================================================================

PROJECT   := rt-thread-industrial-robot-gateway
BUILD     := build
TOPDIR    ?= $(CURDIR)/..

CC        ?= gcc
CSTD      ?= -std=c99
OPT       ?= -O2
PYTHON    ?= python3

WARNINGS  := -Wall -Wextra -Werror -Wno-unused-parameter
CPPFLAGS  += -Iinclude -Iport
CFLAGS    += $(CSTD) $(OPT) -g $(WARNINGS)
LDFLAGS   +=
LDLIBS    += -lpthread

GW_SRCS    := $(wildcard src/*.c)
PORT_SRCS  := port/port_host.c port/port_hw_stub.c
SIM_SRCS   := port/gw_host_sim.c
TEST_SRCS  := $(wildcard test/*.c)

GW_OBJS    := $(patsubst %.c,$(BUILD)/%.o,$(GW_SRCS) $(PORT_SRCS))
SIM_OBJS   := $(GW_OBJS) $(patsubst %.c,$(BUILD)/%.o,$(SIM_SRCS))
TEST_OBJS  := $(GW_OBJS) $(patsubst %.c,$(BUILD)/%.o,$(TEST_SRCS))

TARGET_SIM  := $(BUILD)/gw_host_sim
TARGET_TEST := $(BUILD)/gw_tests

.PHONY: all test verify sim clean rebuild help

all: $(TARGET_SIM)

$(TARGET_SIM): $(SIM_OBJS)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)
	@echo "==> built $@"

$(TARGET_TEST): $(TEST_OBJS)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)
	@echo "==> built $@"

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

# 单元测试：跑不过就返回非 0，可以直接挂到 CI 上
test: $(TARGET_TEST)
	@echo "==> running unit tests"
	@$(TARGET_TEST)

# 端到端协议验证：C 网关 <-> Python 模拟从站
verify: $(TARGET_SIM)
	@echo "==> running end-to-end protocol verification"
	@$(PYTHON) $(TOPDIR)/tools/verify_protocol.py --gateway-bin $(CURDIR)/$(TARGET_SIM)

sim: $(TARGET_SIM)

rebuild: clean all

clean:
	rm -rf $(BUILD)

help:
	@echo "make / make test / make verify / make clean"
	@echo "cross build for RT-Thread target: see gateway/SConscript"

-include $(wildcard $(BUILD)/*.d) $(wildcard $(BUILD)/*/*.d)
