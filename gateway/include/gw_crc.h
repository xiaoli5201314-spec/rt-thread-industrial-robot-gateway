/*
 * gw_crc.h - CRC 校验
 *
 * 提供两种实现：
 *   1) 位运算逐位实现（省 Flash, 适合小容量 MCU 或启动早期使用）
 *   2) 256 项查表实现（时间恒定, 适合高频报文热路径）
 * 两者结果必须一致, 单元测试中会逐字节交叉验证。
 */
#ifndef GW_CRC_H
#define GW_CRC_H

#include "gw_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Modbus CRC16：多项式 0xA001(反射), 初值 0xFFFF, 无最终异或
 * 标准向量: "123456789" -> 0x4B37 */
uint16_t gw_crc16_modbus(const uint8_t *data, size_t len);

/* 分片累加版本：用于环形缓冲中跨两段的连续计算 */
uint16_t gw_crc16_modbus_update(uint16_t crc, const uint8_t *data, size_t len);

/* 查表版本, 结果与 gw_crc16_modbus 完全一致 */
uint16_t gw_crc16_modbus_fast(const uint8_t *data, size_t len);

/* 便利函数：校验一整帧（末两字节为小端 CRC, Modbus 规程要求低字节在前） */
bool gw_crc16_modbus_check_frame(const uint8_t *frame, size_t len);

/* 在帧尾写入 CRC 低字节在前 */
void gw_crc16_modbus_append(uint8_t *frame, size_t len_without_crc);

/* CANopen 的 CRC 不用于数据帧；此处额外提供 CRC8 供参数存储头部使用
 * 多项式 0x07, 初值 0x00, 无反射（与常见 1-Wire/CRC-8 一致） */
uint8_t gw_crc8(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* GW_CRC_H */
