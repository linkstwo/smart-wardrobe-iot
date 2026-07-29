#ifndef AXIS_COORD_H
#define AXIS_COORD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "motor_hw.h"

/* M2 是旋转轴/绝对值编码器相关轴，不属于 XYZ 三轴坐标。 */
#define AXIS_ROT_MOTOR_ID MOTOR_ID_2

typedef enum {
    /* 衣柜三轴坐标：X=M1，Y=M3，Z=M4。 */
    AXIS_ID_X = 0,
    AXIS_ID_Y,
    AXIS_ID_Z,
    AXIS_ID_COUNT,
    AXIS_ID_INVALID = 0xFF
} AxisId_t;

typedef struct {
    /* XYZ 三轴当前位置，单位为步数。 */
    int32_t x;
    int32_t y;
    int32_t z;
} AxisPos_t;

void AxisCoord_Init(void);
/* 每 1ms 根据电机累计步数更新 XYZ 坐标和 M2 旋转轴累计值。 */
void AxisCoord_Tick1ms(void);
/* 读取单轴或三轴坐标。 */
int AxisCoord_GetPos(AxisId_t axis, int32_t *out_steps);
int AxisCoord_GetAll(AxisPos_t *out);
/* 将当前 XYZ 坐标设为 0 点。 */
int AxisCoord_ZeroNow(void);

/* M2 旋转轴角度读取、清零、保存和从 Flash 恢复。 */
int AxisCoord_M3RotGet(int32_t *out_steps, float *out_deg);
int AxisCoord_M3RotZeroNow(void);
int AxisCoord_M3RotSetAndSaveSteps(int32_t steps);
int AxisCoord_M3RotSaveCurrentToFlash(void);
int AxisCoord_M3RotLoadSavedFromFlash(float *out_deg);

/* 电机编号与 XYZ 坐标轴之间的映射转换。 */
AxisId_t AxisCoord_FromMotor(MotorId_t motor_id);
MotorId_t AxisCoord_ToMotor(AxisId_t axis);

#ifdef __cplusplus
}
#endif

#endif
