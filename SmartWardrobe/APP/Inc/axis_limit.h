#ifndef AXIS_LIMIT_H
#define AXIS_LIMIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "axis_coord.h"

typedef struct {
    /* valid 表示 XYZ 最大行程是否已经配置有效。 */
    bool valid;
    /* XYZ 三轴最大允许坐标，单位步数。 */
    uint32_t x_max;
    uint32_t y_max;
    uint32_t z_max;
} AxisLimitConfig_t;

typedef enum {
    /* 限位边界类型：低边界表示小于 0，高边界表示超过最大值。 */
    AXIS_LIMIT_BOUND_NONE = 0,
    AXIS_LIMIT_BOUND_LOW,
    AXIS_LIMIT_BOUND_HIGH
} AxisLimitBound_t;

typedef struct {
    /* XYZ 限位故障锁存信息。 */
    bool active;
    AxisId_t axis;
    AxisLimitBound_t bound;
    int32_t value;
    uint32_t max;
} AxisLimitFault_t;

typedef struct {
    /* M2 旋转轴限位状态。 */
    bool valid;
    uint32_t max_step;
    bool fault_active;
    AxisLimitBound_t fault_bound;
    int32_t fault_value;
} AxisRotLimitStatus_t;

void AxisLimit_Init(void);
/* 1ms 巡检 XYZ 和 M2 旋转轴是否越界。 */
void AxisLimit_Tick1ms(void);
/* 设置并保存 XYZ 最大行程。 */
int AxisLimit_SetAndSave(uint32_t x_max, uint32_t y_max, uint32_t z_max);
/* 查询 XYZ 限位配置和故障。 */
int AxisLimit_Get(AxisLimitConfig_t *out);
int AxisLimit_GetFault(AxisLimitFault_t *out);
/* 设置并保存 M2 旋转轴最大步数。 */
int AxisLimit_SetM3RotMaxAndSave(uint32_t rot_max_step);
int AxisLimit_GetM3RotStatus(AxisRotLimitStatus_t *out);
/* 判断保护开启时限位参数是否已准备好。 */
bool AxisLimit_IsReady(void);
/* 电机动作前的限位检查。 */
int AxisLimit_CanMoveMotor(MotorId_t motor_id);
/* 清除锁存故障，不修改保存参数。 */
void AxisLimit_ClearFault(void);
/* 设置并保存限位保护总开关。 */
int AxisLimit_SetProtectAndSave(bool enable);
bool AxisLimit_IsProtectEnabled(void);

/* 仅修改运行时状态，不写 Flash，保留给兼容旧调用路径使用。 */
void AxisLimit_SetProtectionEnabled(bool enable);
bool AxisLimit_IsProtectionEnabled(void);

const char *AxisLimit_AxisToString(AxisId_t axis);
const char *AxisLimit_BoundToString(AxisLimitBound_t bound);

#ifdef __cplusplus
}
#endif

#endif
