#ifndef AXIS_PLANNER_H
#define AXIS_PLANNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "axis_coord.h"
#include "axis_limit.h"

/* 三轴规划参数保存到 Flash 时，rpm 使用 rpm * 10 的整数形式。 */
#define AXIS_PLAN_RPM_MIN_X10 100U
#define AXIS_PLAN_RPM_MAX_X10 5000U
#define AXIS_PLAN_ERR_MIN 10U
#define AXIS_PLAN_ERR_MAX 20000U
#define AXIS_PLAN_POS_TOL_MIN 1U
#define AXIS_PLAN_POS_TOL_MAX 1000U

typedef enum {
    /* 三轴规划器状态。 */
    AXIS_PLAN_STATE_IDLE = 0,
    AXIS_PLAN_STATE_RUNNING,
    AXIS_PLAN_STATE_DONE,
    AXIS_PLAN_STATE_FAULT
} AxisPlanState_t;

typedef enum {
    /* 三轴规划器故障类型。 */
    AXIS_PLAN_FAULT_NONE = 0,
    AXIS_PLAN_FAULT_LIMIT,
    AXIS_PLAN_FAULT_ENCODER_MISMATCH,
    AXIS_PLAN_FAULT_MOTOR_CMD
} AxisPlanFaultCode_t;

typedef struct {
    /* 默认移动转速，单位 rpm。 */
    float move_rpm_default;
    /* 编码器误差阈值，保留给误差检测/诊断使用。 */
    uint32_t enc_err_max;
    /* 判断到位的坐标容差，单位步数。 */
    uint32_t pos_tol;
} AxisPlanConfig_t;

typedef struct {
    /* 当前状态、当前位置、目标位置和故障信息。 */
    AxisPlanState_t state;
    AxisPos_t current;
    AxisPos_t target;
    AxisPlanFaultCode_t fault_code;
    AxisId_t fault_axis;
    int32_t fault_value;
    uint32_t fault_ref;
    AxisPlanConfig_t cfg;
} AxisPlanStatus_t;

void AxisPlanner_Init(void);
/* 1ms 调度入口，用于判断三轴是否到达目标容差。 */
void AxisPlanner_Tick1ms(void);
/* 移动到指定 XYZ 目标，rpm_hint 为 0 时使用默认速度。 */
int AxisPlanner_MoveTo(int32_t x, int32_t y, int32_t z, float rpm_hint);
/* 普通停止和急停当前规划。 */
int AxisPlanner_Stop(void);
int AxisPlanner_EStop(void);
int AxisPlanner_GetStatus(AxisPlanStatus_t *out);
int AxisPlanner_SetAndSaveConfig(float move_rpm_default, uint32_t enc_err_max, uint32_t pos_tol);
int AxisPlanner_GetConfig(AxisPlanConfig_t *out);

const char *AxisPlanner_StateToString(AxisPlanState_t state);
const char *AxisPlanner_FaultToString(AxisPlanFaultCode_t fault);

#ifdef __cplusplus
}
#endif

#endif
