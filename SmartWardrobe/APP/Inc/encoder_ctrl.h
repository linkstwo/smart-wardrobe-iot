#ifndef ENCODER_CTRL_H
#define ENCODER_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "motor_hw.h"

typedef struct {
    /* 软件累计编码器计数，单位为编码器计数。 */
    int32_t count_total;
    /* 最近判断出的运动方向。 */
    MotorDir_t dir;
    /* 按窗口估算的编码器转速，单位 rpm。 */
    float rpm;
} EncoderStatus_t;

/* 初始化编码器控制层，内部会启动 encoder_hw.c。 */
void EncoderCtrl_Init(void);
/* 1ms 调度入口，用于累计计数和估算转速。 */
void EncoderCtrl_Tick1ms(void);
/* 获取编码器状态快照。 */
int EncoderCtrl_GetStatus(MotorId_t id, EncoderStatus_t *out);
/* 清零单个编码器硬件计数器和软件累计状态。 */
int EncoderCtrl_Clear(MotorId_t id);

#ifdef __cplusplus
}
#endif

#endif
