#include "encoder_ctrl.h"

#include <limits.h>
#include <string.h>
#include "encoder_hw.h"
#include "motor_ctrl.h"

#define ENCODER_SPEED_WINDOW_MS 50U

/*
 * 编码器控制层：
 * encoder_hw.c 只读取硬件计数器，本文件负责处理计数器回绕、累计总计数、方向和 rpm 估算。
 */
typedef struct {
    /* 上一次读取到的硬件原始计数值。 */
    uint32_t raw_prev;
    /* 软件累计计数，使用 int64_t 降低长时间运行溢出的风险。 */
    int64_t count_total;
    /* 速度计算窗口内的累计增量和窗口时间。 */
    int32_t window_delta;
    uint32_t window_ms;
    /* 最近判断出的编码器方向和估算转速。 */
    MotorDir_t dir;
    float rpm;
} EncoderRuntime_t;

static EncoderRuntime_t g_encoder[MOTOR_COUNT];

static uint8_t motor_id_valid(MotorId_t id)
{
    return (id < MOTOR_COUNT) ? 1U : 0U;
}

static int32_t encoder_delta(uint32_t now, uint32_t prev, uint8_t bits)
{
    /*
     * 16 位计数器用 int16_t 差值处理回绕；
     * 32 位计数器直接用 32 位差值。
     */
    if (bits == 16U) {
        uint16_t n = (uint16_t)now;
        uint16_t p = (uint16_t)prev;
        return (int32_t)(int16_t)(uint16_t)(n - p);
    }

    return (int32_t)(now - prev);
}

void EncoderCtrl_Init(void)
{
    uint32_t i;

    /* 先启动底层编码器，再把当前硬件计数作为软件累计的起点。 */
    EncoderHw_Init();
    memset(g_encoder, 0, sizeof(g_encoder));

    for (i = 0U; i < MOTOR_COUNT; i++) {
        g_encoder[i].raw_prev = EncoderHw_ReadCounter((MotorId_t)i);
        g_encoder[i].dir = MOTOR_DIR_CW;
        g_encoder[i].rpm = 0.0f;
    }
}

void EncoderCtrl_Tick1ms(void)
{
    uint32_t i;

    /*
     * 每 1ms 更新一次编码器状态。
     * rpm 不按每个 1ms 立即计算，而是在 50ms 窗口内累计，减少抖动。
     */
    for (i = 0U; i < MOTOR_COUNT; i++) {
        EncoderRuntime_t *e = &g_encoder[i];
        MotorStatusSnapshot_t motor_st;
        uint32_t now_raw;
        int32_t delta;

        /* 读取原始计数并换算成考虑回绕后的增量。 */
        now_raw = EncoderHw_ReadCounter((MotorId_t)i);
        delta = encoder_delta(now_raw, e->raw_prev, EncoderHw_GetCounterBits((MotorId_t)i));
        e->raw_prev = now_raw;

        /* 增量正负先表示逻辑坐标变化，再按每轴配置换算成真实电机方向。 */
        if (delta > 0) {
            e->dir = MotorHw_GetDirectionForPositionChange((MotorId_t)i, true);
        } else if (delta < 0) {
            e->dir = MotorHw_GetDirectionForPositionChange((MotorId_t)i, false);
        } else {
            e->dir = MotorHw_GetDirectionForPositionChange(
                (MotorId_t)i,
                !EncoderHw_IsCountingDown((MotorId_t)i));
        }

        e->count_total += (int64_t)delta;
        e->window_delta += delta;
        e->window_ms++;

        /* 到达速度窗口后，根据窗口内脉冲增量估算 rpm。 */
        if (e->window_ms >= ENCODER_SPEED_WINDOW_MS) {
            uint32_t abs_delta;

            abs_delta = (e->window_delta >= 0) ? (uint32_t)e->window_delta : (uint32_t)(-e->window_delta);
            if ((MotorCtrl_GetStatus((MotorId_t)i, &motor_st) == 0) && (motor_st.ppr > 0U)) {
                e->rpm = (float)((double)abs_delta * (60000.0 / (double)e->window_ms) / (double)motor_st.ppr);
            } else {
                e->rpm = 0.0f;
            }

            e->window_delta = 0;
            e->window_ms = 0U;
        }
    }
}

int EncoderCtrl_GetStatus(MotorId_t id, EncoderStatus_t *out)
{
    uint32_t primask;

    /* 返回编码器状态快照，count_total 超出 int32_t 时做饱和处理。 */
    if ((!motor_id_valid(id)) || (out == 0)) {
        return -1;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (g_encoder[id].count_total > (int64_t)INT32_MAX) {
        out->count_total = INT32_MAX;
    } else if (g_encoder[id].count_total < (int64_t)INT32_MIN) {
        out->count_total = INT32_MIN;
    } else {
        out->count_total = (int32_t)g_encoder[id].count_total;
    }
    out->dir = g_encoder[id].dir;
    out->rpm = g_encoder[id].rpm;
    if (primask == 0U) {
        __enable_irq();
    }

    return 0;
}

int EncoderCtrl_Clear(MotorId_t id)
{
    uint32_t primask;

    /* 清零硬件计数器和软件累计状态，常用于回零或重新标定后。 */
    if (!motor_id_valid(id)) {
        return -1;
    }

    if (EncoderHw_Clear(id) != 0) {
        return -1;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    g_encoder[id].raw_prev = EncoderHw_ReadCounter(id);
    g_encoder[id].count_total = 0;
    g_encoder[id].window_delta = 0;
    g_encoder[id].window_ms = 0U;
    g_encoder[id].rpm = 0.0f;
    if (primask == 0U) {
        __enable_irq();
    }

    return 0;
}
