#ifndef MOTOR_HW_H
#define MOTOR_HW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "tim.h"

/* 项目中共有 M1~M5 五个步进电机。 */
#define MOTOR_COUNT 5U

/* STEP 脉冲频率范围和统一计数频率，motor_hw.c 会把定时器分频到 1MHz。 */
#define MOTOR_STEP_FREQ_MIN_HZ 20U
#define MOTOR_STEP_FREQ_MAX_HZ 8000U
#define MOTOR_COUNTER_FREQ_HZ 1000000U

typedef enum {
    /* 电机编号必须和 motor_hw.c 中的硬件映射数组下标保持一致。 */
    MOTOR_ID_1 = 0,
    MOTOR_ID_2 = 1,
    MOTOR_ID_3 = 2,
    MOTOR_ID_4 = 3,
    MOTOR_ID_5 = 4,
    MOTOR_ID_INVALID = 0xFF
} MotorId_t;

typedef enum {
    /* 电机方向：CW/CCW 最终会转换为方向 GPIO 电平。 */
    MOTOR_DIR_CW = 1,
    MOTOR_DIR_CCW = 2
} MotorDir_t;

typedef struct {
    /* 输出 STEP PWM 的定时器句柄和通道。 */
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    /* 定时器输入时钟，用于计算预分频。 */
    uint32_t timer_clk_hz;
    /* 方向引脚。 */
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    /* 使能引脚。 */
    GPIO_TypeDef *en_port;
    uint16_t en_pin;
} MotorHwConfig_t;

/* 初始化 M1~M5 硬件输出、定时器分频、默认方向和使能状态。 */
void MotorHw_Init(void);
/* 获取某个电机的硬件配置，只读使用。 */
const MotorHwConfig_t *MotorHw_GetConfig(MotorId_t id);
/* 获取真实运动方向与逻辑坐标/编码器计数增减之间的映射。 */
MotorDir_t MotorHw_GetPositiveDirection(MotorId_t id);
bool MotorHw_DirectionIncreasesPosition(MotorId_t id, MotorDir_t dir);
MotorDir_t MotorHw_GetDirectionForPositionChange(MotorId_t id, bool increase);
/* 根据定时器句柄反查电机编号，供定时器更新中断计步使用。 */
MotorId_t MotorHw_FromTim(TIM_HandleTypeDef *htim);

/* 设置方向、使能、启动/停止 STEP PWM。 */
void MotorHw_SetDirection(MotorId_t id, MotorDir_t dir);
void MotorHw_SetEnable(MotorId_t id, bool enable);
void MotorHw_StartPulse(MotorId_t id);
void MotorHw_StopPulse(MotorId_t id);
/* 设置 STEP 脉冲频率，单位 Hz。 */
bool MotorHw_SetStepFreq(MotorId_t id, uint32_t step_hz);

#ifdef __cplusplus
}
#endif

#endif
