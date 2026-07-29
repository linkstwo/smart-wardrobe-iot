#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "motor_hw.h"

/* 默认每转脉冲数和允许范围，用于 rpm 与 step_hz 换算。 */
#define MOTOR_PPR_DEFAULT 3200U
#define MOTOR_PPR_MIN 100U
#define MOTOR_PPR_MAX 50000U

/* 默认加减速度和允许范围，单位 rpm/s。 */
#define MOTOR_ACC_DEFAULT_RPM_S 120.0f
#define MOTOR_DEC_DEFAULT_RPM_S 120.0f
#define MOTOR_ACC_DEC_MIN_RPM_S 1.0f
#define MOTOR_ACC_DEC_MAX_RPM_S 5000.0f

/* 普通停止的默认软停止时间和允许范围，单位 ms。 */
#define MOTOR_SOFT_STOP_DEFAULT_MS 2500U
#define MOTOR_SOFT_STOP_MIN_MS 100U
#define MOTOR_SOFT_STOP_MAX_MS 30000U

typedef enum {
    /* 电机控制状态机模式。 */
    MOTOR_MODE_IDLE = 0,
    MOTOR_MODE_RUN,
    MOTOR_MODE_RAMP_UP,
    MOTOR_MODE_RAMP_DOWN,
    MOTOR_MODE_MOVE_STEPS,
    MOTOR_MODE_RECIP_FINITE,
    MOTOR_MODE_RECIP_INFINITE,
    MOTOR_MODE_SOFT_STOP,
    MOTOR_MODE_ESTOP
} MotorMode_t;

typedef struct {
    /* 当前模式和方向。 */
    MotorMode_t mode;
    MotorDir_t dir;
    /* 当前每转脉冲数。 */
    uint32_t ppr;
    /* 当前转速和目标转速，单位 rpm。 */
    float rpm_current;
    float rpm_target;
    /* 加速度、减速度和软停止时间。 */
    float acc_rpm_s;
    float dec_rpm_s;
    uint32_t soft_stop_ms;
    /* 累计步数和指定步数运动剩余步数。 */
    uint32_t steps_total;
    uint32_t steps_remaining;
    /* 往返运动总次数和已完成次数。 */
    uint32_t recip_cycles;
    uint32_t recip_cycles_done;
} MotorStatusSnapshot_t;

/* 初始化电机控制层，内部会初始化 motor_hw.c。 */
void MotorCtrl_Init(void);
/* 1ms 调度入口，用于更新加减速、软停止等状态。 */
void MotorCtrl_Tick1ms(void);
/* 定时器更新中断入口，用于累计 STEP 脉冲数。 */
void MotorCtrl_OnTimUpdate(TIM_HandleTypeDef *htim);

/* 连续运行、停止、急停和改速接口。 */
int MotorCtrl_Run(MotorId_t id, MotorDir_t dir, float rpm);
int MotorCtrl_Stop(MotorId_t id, uint32_t soft_stop_ms);
int MotorCtrl_EStop(MotorId_t id);
int MotorCtrl_SetSpeed(MotorId_t id, float rpm);
int MotorCtrl_AccelTo(MotorId_t id, float to_rpm, uint32_t duration_ms);
int MotorCtrl_DecelTo(MotorId_t id, float to_rpm, uint32_t duration_ms);
/* 指定步数运动和往返运动接口。 */
int MotorCtrl_MoveSteps(MotorId_t id, MotorDir_t dir, float rpm, uint32_t steps);
int MotorCtrl_Recip(MotorId_t id, MotorDir_t dir, float rpm, uint32_t steps, uint32_t cycles);
int MotorCtrl_RecipInfinite(MotorId_t id, MotorDir_t dir, float rpm, uint32_t steps);
/* 修改 ppr、加速度、减速度、软停止时间，set_xxx 为 true 才更新对应字段。 */
int MotorCtrl_Configure(MotorId_t id,
                        bool set_ppr, uint32_t ppr,
                        bool set_acc, float acc_rpm_s,
                        bool set_dec, float dec_rpm_s,
                        bool set_stop, uint32_t soft_stop_ms);

/* 对全部电机普通停止或急停。 */
void MotorCtrl_StopAll(uint32_t soft_stop_ms);
void MotorCtrl_EStopAll(void);

/* 获取电机状态快照和 rpm 边界，供命令响应和其他模块判断。 */
int MotorCtrl_GetStatus(MotorId_t id, MotorStatusSnapshot_t *out_status);
float MotorCtrl_GetRpmMin(uint32_t ppr);
float MotorCtrl_GetRpmMax(uint32_t ppr);
const char *MotorCtrl_ModeToString(MotorMode_t mode);
const char *MotorCtrl_DirToString(MotorDir_t dir);

#ifdef __cplusplus
}
#endif

#endif
