# ============================================================================
# RT-Thread 工业通信网关 - 仓库根 Makefile（转发到 gateway/）
#
# 所有实际构建规则在 gateway/Makefile 中，这里只是给习惯从根目录
# 敲 make 的人一个入口，保持两种用法都可用：
#   make test          -> gateway/Makefile 的 test
#   make -C gateway test
# ============================================================================

.PHONY: all sim test verify clean rebuild help

all:
	$(MAKE) -C gateway all

sim:
	$(MAKE) -C gateway sim

test:
	$(MAKE) -C gateway test

verify:
	$(MAKE) -C gateway verify

clean:
	$(MAKE) -C gateway clean

rebuild: clean all

help:
	@echo "make           编译 PC 端整机仿真程序 build/gw_host_sim"
	@echo "make test      编译并运行全部单元测试"
	@echo "make verify    运行 tools/verify_protocol.py 端到端协议验证"
	@echo "make clean     清理构建产物"
	@echo ""
	@echo "目标板（RT-Thread）请用 scons 构建，见 gateway/SConscript"
