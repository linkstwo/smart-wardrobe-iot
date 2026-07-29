#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 运动曲线模块接口。
 * 当前主要给 motor_ctrl.c 的软停止使用，通过余弦衰减系数让 STEP 频率平滑降下来，
 * 避免直接断脉冲造成机械冲击。
 */

/* 软停止曲线查表长度，表越长曲线越细，但占用的常量空间也越多。 */
#define MOTION_SINE_TABLE_SIZE 128U

/*
 * 获取 Q15 格式余弦衰减系数，用于软停止平滑降速。
 * elapsed_ms 是已经经过的停止时间，total_ms 是本次软停止总时间。
 */
uint16_t MotionProfile_GetCosineDecayQ15(uint32_t elapsed_ms, uint32_t total_ms);

#ifdef __cplusplus
}
#endif

#endif
