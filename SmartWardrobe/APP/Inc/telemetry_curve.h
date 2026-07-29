#ifndef TELEMETRY_CURVE_H
#define TELEMETRY_CURVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "motor_hw.h"

/* 曲线遥测输出频率默认值和范围，单位 Hz。 */
#define CURVE_RATE_DEFAULT_HZ 20U
#define CURVE_RATE_MIN_HZ 1U
#define CURVE_RATE_MAX_HZ 50U

typedef struct {
    /* 是否开启曲线输出。 */
    bool enabled;
    /* 当前输出频率。 */
    uint16_t rate_hz;
    /* 因发送队列满等原因丢弃的曲线消息数量。 */
    uint32_t lost;
} CurveStatus_t;

/* 初始化曲线遥测模块。 */
void TelemetryCurve_Init(void);
/* 1ms 调度入口，按配置频率输出曲线数据。 */
void TelemetryCurve_Tick1ms(void);
/* 设置单个电机或全部电机曲线输出。 */
int TelemetryCurve_Set(MotorId_t id, bool enable, uint16_t rate_hz);
void TelemetryCurve_SetAll(bool enable, uint16_t rate_hz);
int TelemetryCurve_GetStatus(MotorId_t id, CurveStatus_t *out);

#ifdef __cplusplus
}
#endif

#endif
