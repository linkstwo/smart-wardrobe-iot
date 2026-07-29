#ifndef ENCODER_HW_H
#define ENCODER_HW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "motor_hw.h"

/* 增量编码器硬件接口：负责启动编码器定时器、读取原始计数和 AB 相电平。 */
void EncoderHw_Init(void);
/* 读取硬件原始计数器。 */
uint32_t EncoderHw_ReadCounter(MotorId_t id);
/* 读取硬件方向位。 */
bool EncoderHw_IsCountingDown(MotorId_t id);
/* 获取计数器位宽，控制层用它处理 16/32 位回绕。 */
uint8_t EncoderHw_GetCounterBits(MotorId_t id);
/* 清零硬件计数器。 */
int EncoderHw_Clear(MotorId_t id);
/* 读取编码器 A/B 相电平，主要用于调试接线。 */
int EncoderHw_ReadAB(MotorId_t id, bool *a_high, bool *b_high);

#ifdef __cplusplus
}
#endif

#endif
