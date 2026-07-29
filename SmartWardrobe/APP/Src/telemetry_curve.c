#include "telemetry_curve.h"

#include <stdio.h>
#include <string.h>
#include "encoder_ctrl.h"
#include "motor_ctrl.h"
#include "uart_console.h"

/*
 * 曲线遥测模块：
 * 按配置频率周期性输出电机目标转速、控制转速、编码器转速和模式，
 * 方便上位机画曲线或调试速度闭环/开环表现。
 */
typedef struct {
    uint8_t enabled;
    uint16_t rate_hz;
    uint16_t period_ms;
    uint16_t elapsed_ms;
    uint32_t lost;
} CurveRuntime_t;

static CurveRuntime_t g_curve[MOTOR_COUNT];

static uint8_t motor_id_valid(MotorId_t id)
{
    return (id < MOTOR_COUNT) ? 1U : 0U;
}

static uint16_t curve_rate_to_period(uint16_t rate_hz)
{
    uint16_t period;

    /* 将输出频率 Hz 转换成毫秒周期，并限制在允许的最小/最大频率内。 */
    if (rate_hz < CURVE_RATE_MIN_HZ) {
        rate_hz = CURVE_RATE_MIN_HZ;
    } else if (rate_hz > CURVE_RATE_MAX_HZ) {
        rate_hz = CURVE_RATE_MAX_HZ;
    }

    period = (uint16_t)(1000U / rate_hz);
    if (period == 0U) {
        period = 1U;
    }
    return period;
}

static void curve_emit(MotorId_t id)
{
    MotorStatusSnapshot_t m;
    EncoderStatus_t e;
    bool dropped = false;
    char line[192];

    /* 同时读取电机控制状态和编码器状态，组成一行文本遥测。 */
    if (MotorCtrl_GetStatus(id, &m) != 0) {
        return;
    }
    if (EncoderCtrl_GetStatus(id, &e) != 0) {
        return;
    }

    (void)snprintf(line,
                   sizeof(line),
                   "CURVE M=%u TS=%lu TARGET=%.2f CTRL=%.2f ENC=%.2f MODE=%s",
                   (unsigned int)(id + 1U),
                   (unsigned long)HAL_GetTick(),
                   m.rpm_target,
                   m.rpm_current,
                   e.rpm,
                   MotorCtrl_ModeToString(m.mode));

    /*
     * 曲线数据频率可能较高，串口队列满时允许丢弃旧数据。
     * lost 用于记录丢弃次数，方便判断上位机是否接收不过来。
     */
    if (!UartConsole_QueueLineDropOldest(line, &dropped)) {
        g_curve[id].lost++;
        return;
    }

    if (dropped) {
        g_curve[id].lost++;
    }
}

void TelemetryCurve_Init(void)
{
    uint32_t i;

    /* 默认关闭曲线输出，但为每个电机预设默认输出频率。 */
    memset(g_curve, 0, sizeof(g_curve));
    for (i = 0U; i < MOTOR_COUNT; i++) {
        g_curve[i].enabled = 0U;
        g_curve[i].rate_hz = CURVE_RATE_DEFAULT_HZ;
        g_curve[i].period_ms = curve_rate_to_period(CURVE_RATE_DEFAULT_HZ);
        g_curve[i].elapsed_ms = 0U;
        g_curve[i].lost = 0U;
    }
}

void TelemetryCurve_Tick1ms(void)
{
    uint32_t i;

    /* 每 1ms 检查一次各电机曲线输出周期。 */
    for (i = 0U; i < MOTOR_COUNT; i++) {
        CurveRuntime_t *c = &g_curve[i];

        if (c->enabled == 0U) {
            continue;
        }

        c->elapsed_ms++;
        if (c->elapsed_ms >= c->period_ms) {
            c->elapsed_ms = 0U;
            curve_emit((MotorId_t)i);
        }
    }
}

int TelemetryCurve_Set(MotorId_t id, bool enable, uint16_t rate_hz)
{
    /* 设置单个电机曲线输出开关和频率，rate_hz 为 0 时使用默认频率。 */
    if (!motor_id_valid(id)) {
        return -1;
    }

    if (enable) {
        if (rate_hz == 0U) {
            rate_hz = CURVE_RATE_DEFAULT_HZ;
        }
        if ((rate_hz < CURVE_RATE_MIN_HZ) || (rate_hz > CURVE_RATE_MAX_HZ)) {
            return -2;
        }

        g_curve[id].enabled = 1U;
        g_curve[id].rate_hz = rate_hz;
        g_curve[id].period_ms = curve_rate_to_period(rate_hz);
        g_curve[id].elapsed_ms = 0U;
    } else {
        g_curve[id].enabled = 0U;
        if (rate_hz != 0U) {
            if ((rate_hz < CURVE_RATE_MIN_HZ) || (rate_hz > CURVE_RATE_MAX_HZ)) {
                return -2;
            }
            g_curve[id].rate_hz = rate_hz;
            g_curve[id].period_ms = curve_rate_to_period(rate_hz);
        }
    }

    return 0;
}

void TelemetryCurve_SetAll(bool enable, uint16_t rate_hz)
{
    uint32_t i;

    /* 批量设置所有电机曲线输出，用于上位机一键开启/关闭遥测。 */
    for (i = 0U; i < MOTOR_COUNT; i++) {
        (void)TelemetryCurve_Set((MotorId_t)i, enable, rate_hz);
    }
}

int TelemetryCurve_GetStatus(MotorId_t id, CurveStatus_t *out)
{
    /* 查询曲线输出状态和丢包计数。 */
    if ((!motor_id_valid(id)) || (out == 0)) {
        return -1;
    }

    out->enabled = (g_curve[id].enabled != 0U);
    out->rate_hz = g_curve[id].rate_hz;
    out->lost = g_curve[id].lost;
    return 0;
}
