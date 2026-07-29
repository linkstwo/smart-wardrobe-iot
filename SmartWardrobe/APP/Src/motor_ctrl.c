#include "motor_ctrl.h"
#include <string.h>
#include "motion_profile.h"

/*
 * 单个电机的软件运行时状态。
 * 这个结构体不直接保存 GPIO/定时器句柄，硬件资源统一由 motor_hw.c 管理。
 * 本模块负责把“运行、指定步数、往返、软停止、急停”等控制意图转换成硬件输出。
 *
 * 重要说明：
 * 本模块没有实现 PID。rpm_cur/rpm_target 只是目标转速和当前输出转速，
 * Tick1ms 里按固定加减速或软停止曲线更新 STEP 频率，没有使用编码器误差做 Kp/Ki/Kd 闭环计算。
 */
typedef struct {
    /* 当前电机运行模式，决定 Tick1ms 和定时器中断中的处理分支。 */
    MotorMode_t mode;
    /* 当前方向，最终会写到 DIR 引脚。 */
    MotorDir_t dir;
    /* 每转脉冲数，用于 rpm 和 step_hz 互相换算。 */
    uint32_t ppr;
    /* 当前转速和目标转速，单位为 rpm。 */
    float rpm_cur;
    float rpm_target;
    /* 加速度、减速度，单位为 rpm/s。 */
    float acc_rpm_s;
    float dec_rpm_s;
    /* 默认软停止时间，单位 ms。 */
    uint32_t soft_stop_ms;

    /* steps_total 为累计脉冲数，step_remaining 用于 MOVE_STEPS/往返运动剩余步数。 */
    uint32_t steps_total;
    uint32_t step_remaining;

    /* 往返运动参数：单程步数、总往返次数、已完成次数、当前是否处在返回段。 */
    uint32_t recip_steps;
    uint32_t recip_cycles;
    uint32_t recip_cycles_done;
    uint8_t recip_return_leg;

    /* 定时加减速过程参数。 */
    float ramp_target_rpm;
    float ramp_start_rpm;
    uint32_t ramp_total_ms;
    uint32_t ramp_elapsed_ms;

    /* 软停止过程参数，配合 motion_profile.c 的曲线做平滑降速。 */
    float stop_start_rpm;
    uint32_t stop_total_ms;
    uint32_t stop_elapsed_ms;

    /* 已经写入硬件的 step_hz，避免每个 Tick 重复写 ARR/CCR。 */
    uint32_t applied_freq_hz;
} MotorRuntime_t;

/* 五个电机的软件状态数组，下标和 MotorId_t 保持一致。 */
static MotorRuntime_t g_motor[MOTOR_COUNT];

static uint8_t motor_id_valid(MotorId_t id)
{
    return (id < MOTOR_COUNT) ? 1U : 0U;
}

float MotorCtrl_GetRpmMin(uint32_t ppr)
{
    /* 最低 rpm 由硬件允许的最低 STEP 频率和每转脉冲数换算得到。 */
    if (ppr == 0U) {
        return 0.0f;
    }
    return (60.0f * (float)MOTOR_STEP_FREQ_MIN_HZ) / (float)ppr;
}

float MotorCtrl_GetRpmMax(uint32_t ppr)
{
    /* 最高 rpm 由硬件允许的最高 STEP 频率和每转脉冲数换算得到。 */
    if (ppr == 0U) {
        return 0.0f;
    }
    return (60.0f * (float)MOTOR_STEP_FREQ_MAX_HZ) / (float)ppr;
}

static uint8_t motor_rpm_valid(const MotorRuntime_t *m, float rpm)
{
    /* 这里只检查基本合法性，具体上下限在换算 step_hz 时再统一夹紧。 */
    if ((m == 0) || (rpm <= 0.0f)) {
        return 0U;
    }
    return 1U;
}

static float motor_rpm_clamp(const MotorRuntime_t *m, float rpm)
{
    float min_rpm;
    float max_rpm;

    if (m == 0) {
        return 0.0f;
    }

    /* 把请求转速限制在当前 ppr 对应的硬件可输出范围内。 */
    min_rpm = MotorCtrl_GetRpmMin(m->ppr);
    max_rpm = MotorCtrl_GetRpmMax(m->ppr);

    if (rpm < min_rpm) {
        return min_rpm;
    }
    if (rpm > max_rpm) {
        return max_rpm;
    }
    return rpm;
}

static uint32_t motor_rpm_to_step_hz(const MotorRuntime_t *m, float rpm)
{
    float hz_f;
    uint32_t hz;

    if ((m == 0) || (rpm <= 0.0f)) {
        return 0U;
    }

    /*
     * rpm 转 STEP 频率：
     * step_hz = rpm * ppr / 60。
     * 结果再限制在 MotorHw_SetStepFreq() 支持的频率范围内。
     */
    hz_f = rpm * (float)m->ppr / 60.0f;
    if (hz_f < (float)MOTOR_STEP_FREQ_MIN_HZ) {
        hz_f = (float)MOTOR_STEP_FREQ_MIN_HZ;
    } else if (hz_f > (float)MOTOR_STEP_FREQ_MAX_HZ) {
        hz_f = (float)MOTOR_STEP_FREQ_MAX_HZ;
    }

    hz = (uint32_t)(hz_f + 0.5f);
    if (hz < MOTOR_STEP_FREQ_MIN_HZ) {
        hz = MOTOR_STEP_FREQ_MIN_HZ;
    } else if (hz > MOTOR_STEP_FREQ_MAX_HZ) {
        hz = MOTOR_STEP_FREQ_MAX_HZ;
    }
    return hz;
}

static void motor_hw_stop(MotorId_t id, MotorRuntime_t *m)
{
    /* 底层停止：关闭 PWM 和使能，并清除会影响下一次运动的动态状态。 */
    MotorHw_StopPulse(id);
    MotorHw_SetEnable(id, false);
    m->applied_freq_hz = 0U;
    m->rpm_cur = 0.0f;
    m->rpm_target = 0.0f;
    m->step_remaining = 0U;
    m->ramp_total_ms = 0U;
    m->ramp_elapsed_ms = 0U;
    m->stop_total_ms = 0U;
    m->stop_elapsed_ms = 0U;
}

static void motor_set_idle(MotorId_t id, MotorRuntime_t *m)
{
    /* 进入空闲态时，除了停止硬件输出，还要清掉往返运动的阶段状态。 */
    motor_hw_stop(id, m);
    m->mode = MOTOR_MODE_IDLE;
    m->recip_steps = 0U;
    m->recip_cycles = 0U;
    m->recip_cycles_done = 0U;
    m->recip_return_leg = 0U;
}

static void motor_prepare_output(MotorId_t id, MotorRuntime_t *m)
{
    /* 按当前方向和使能状态准备硬件输出，真正频率在 motor_apply_output() 中写入。 */
    MotorHw_SetDirection(id, m->dir);
    MotorHw_SetEnable(id, true);
    MotorHw_StartPulse(id);
}

static void motor_start_soft_stop(MotorRuntime_t *m, uint32_t override_ms)
{
    uint32_t t = override_ms;

    /* override_ms 为 0 时使用电机自己的默认软停止时间。 */
    if (t == 0U) {
        t = m->soft_stop_ms;
    }
    if (t < MOTOR_SOFT_STOP_MIN_MS) {
        t = MOTOR_SOFT_STOP_MIN_MS;
    } else if (t > MOTOR_SOFT_STOP_MAX_MS) {
        t = MOTOR_SOFT_STOP_MAX_MS;
    }

    /*
     * 软停止不是立即关 PWM，而是记录起始转速和总时间，
     * 后续每 1ms 用余弦衰减曲线逐步降低 rpm_cur。
     */
    m->mode = MOTOR_MODE_SOFT_STOP;
    m->stop_start_rpm = m->rpm_cur;
    if (m->stop_start_rpm <= 0.0f) {
        m->stop_start_rpm = m->rpm_target;
    }
    if (m->stop_start_rpm <= 0.0f) {
        m->stop_start_rpm = MotorCtrl_GetRpmMin(m->ppr);
    }
    m->stop_total_ms = t;
    m->stop_elapsed_ms = 0U;
    m->rpm_target = 0.0f;
    m->step_remaining = 0U;
    m->ramp_total_ms = 0U;
    m->ramp_elapsed_ms = 0U;
}

static void motor_update_rate(MotorRuntime_t *m)
{
    float delta;

    /* 按固定加速度/减速度逐毫秒逼近目标转速。 */
    if (m->rpm_cur < m->rpm_target) {
        delta = m->acc_rpm_s / 1000.0f;
        m->rpm_cur += delta;
        if (m->rpm_cur > m->rpm_target) {
            m->rpm_cur = m->rpm_target;
        }
    } else if (m->rpm_cur > m->rpm_target) {
        delta = m->dec_rpm_s / 1000.0f;
        m->rpm_cur -= delta;
        if (m->rpm_cur < m->rpm_target) {
            m->rpm_cur = m->rpm_target;
        }
    }

    if ((m->rpm_target <= 0.0f) && (m->rpm_cur <= 0.0f)) {
        m->mode = MOTOR_MODE_IDLE;
    }
}

static void motor_update_timed_ramp(MotorRuntime_t *m)
{
    float ratio;

    /* 定时斜坡用于在指定 duration_ms 内从起始转速线性过渡到目标转速。 */
    if (m->ramp_total_ms == 0U) {
        m->rpm_cur = m->ramp_target_rpm;
        m->rpm_target = m->ramp_target_rpm;
        m->mode = MOTOR_MODE_RUN;
        return;
    }

    if (m->ramp_elapsed_ms < m->ramp_total_ms) {
        m->ramp_elapsed_ms++;
    }

    /* ratio 表示当前加减速进度，范围限制在 0~1。 */
    ratio = (float)m->ramp_elapsed_ms / (float)m->ramp_total_ms;
    if (ratio > 1.0f) {
        ratio = 1.0f;
    }

    m->rpm_cur = m->ramp_start_rpm + (m->ramp_target_rpm - m->ramp_start_rpm) * ratio;

    if (m->ramp_elapsed_ms >= m->ramp_total_ms) {
        m->rpm_cur = m->ramp_target_rpm;
        m->rpm_target = m->ramp_target_rpm;
        m->mode = MOTOR_MODE_RUN;
    }
}

static void motor_update_soft_stop(MotorRuntime_t *m)
{
    uint16_t q15;

    /* 软停止时间为 0 或已经到期时，直接转为空闲。 */
    if (m->stop_total_ms == 0U) {
        m->rpm_cur = 0.0f;
        m->mode = MOTOR_MODE_IDLE;
        return;
    }

    if (m->stop_elapsed_ms >= m->stop_total_ms) {
        m->rpm_cur = 0.0f;
        m->mode = MOTOR_MODE_IDLE;
        return;
    }

    /* q15 从接近 32767 逐渐衰减到 0，用于平滑降低当前转速。 */
    q15 = MotionProfile_GetCosineDecayQ15(m->stop_elapsed_ms, m->stop_total_ms);
    m->rpm_cur = m->stop_start_rpm * (float)q15 / 32767.0f;
    m->stop_elapsed_ms++;
}

static void motor_apply_output(MotorId_t id, MotorRuntime_t *m)
{
    uint32_t hz;

    /* IDLE 和 ESTOP 都会关闭硬件输出，区别是 ESTOP 状态会保留故障语义。 */
    if (m->mode == MOTOR_MODE_IDLE) {
        motor_set_idle(id, m);
        return;
    }

    if (m->mode == MOTOR_MODE_ESTOP) {
        motor_hw_stop(id, m);
        return;
    }

    if (m->rpm_cur <= 0.0f) {
        if (m->mode == MOTOR_MODE_SOFT_STOP) {
            if (m->stop_elapsed_ms >= m->stop_total_ms) {
                motor_set_idle(id, m);
            }
        } else {
            motor_set_idle(id, m);
        }
        return;
    }

    /* 只有 rpm_cur 大于 0 时才真正打开使能和 PWM。 */
    motor_prepare_output(id, m);
    hz = motor_rpm_to_step_hz(m, m->rpm_cur);
    if ((hz != 0U) && (hz != m->applied_freq_hz)) {
        if (MotorHw_SetStepFreq(id, hz)) {
            m->applied_freq_hz = hz;
        }
    }
}

static void motor_handle_recip_leg_done(MotorId_t id, MotorRuntime_t *m)
{
    /*
     * 往返运动每完成一个单程就切换方向。
     * 有限往返在返回段完成后累计 cycles_done，达到目标次数后进入软停止。
     */
    if (m->mode == MOTOR_MODE_RECIP_INFINITE) {
        m->recip_return_leg = m->recip_return_leg ? 0U : 1U;
    } else if (m->mode == MOTOR_MODE_RECIP_FINITE) {
        if (m->recip_return_leg) {
            m->recip_cycles_done++;
            m->recip_return_leg = 0U;
        } else {
            m->recip_return_leg = 1U;
        }

        if (m->recip_cycles_done >= m->recip_cycles) {
            motor_start_soft_stop(m, 0U);
            return;
        }
    } else {
        return;
    }

    m->dir = (m->dir == MOTOR_DIR_CW) ? MOTOR_DIR_CCW : MOTOR_DIR_CW;
    MotorHw_SetDirection(id, m->dir);
    m->step_remaining = m->recip_steps;
}

void MotorCtrl_Init(void)
{
    uint32_t i;

    /* 先初始化硬件层，再清空软件状态，保证上电后所有电机处于停止且失能状态。 */
    MotorHw_Init();

    memset(g_motor, 0, sizeof(g_motor));
    for (i = 0; i < MOTOR_COUNT; i++) {
        g_motor[i].mode = MOTOR_MODE_IDLE;
        g_motor[i].dir = MOTOR_DIR_CW;
        g_motor[i].ppr = MOTOR_PPR_DEFAULT;
        g_motor[i].acc_rpm_s = MOTOR_ACC_DEFAULT_RPM_S;
        g_motor[i].dec_rpm_s = MOTOR_DEC_DEFAULT_RPM_S;
        g_motor[i].soft_stop_ms = MOTOR_SOFT_STOP_DEFAULT_MS;
        motor_set_idle((MotorId_t)i, &g_motor[i]);
    }
}

void MotorCtrl_Tick1ms(void)
{
    uint32_t i;

    /*
     * 1ms 调度入口：
     * - 普通运行和指定步数运动保持目标转速；
     * - 定时加减速更新 rpm_cur；
     * - 软停止按曲线降速；
     * - 急停持续确保硬件输出关闭。
     */
    for (i = 0; i < MOTOR_COUNT; i++) {
        MotorRuntime_t *m = &g_motor[i];

        switch (m->mode) {
            case MOTOR_MODE_IDLE:
                break;
            case MOTOR_MODE_RUN:
            case MOTOR_MODE_MOVE_STEPS:
            case MOTOR_MODE_RECIP_FINITE:
            case MOTOR_MODE_RECIP_INFINITE:
                m->rpm_cur = m->rpm_target;
                break;
            case MOTOR_MODE_RAMP_UP:
            case MOTOR_MODE_RAMP_DOWN:
                motor_update_timed_ramp(m);
                break;
            case MOTOR_MODE_SOFT_STOP:
                motor_update_soft_stop(m);
                break;
            case MOTOR_MODE_ESTOP:
                motor_hw_stop((MotorId_t)i, m);
                break;
            default:
                m->mode = MOTOR_MODE_IDLE;
                break;
        }

        motor_apply_output((MotorId_t)i, m);
    }
}

void MotorCtrl_OnTimUpdate(TIM_HandleTypeDef *htim)
{
    MotorId_t id;
    MotorRuntime_t *m;

    /*
     * 每个 STEP PWM 周期触发一次定时器更新中断。
     * 这里用更新中断作为软件计步来源，累计 steps_total 并扣减 step_remaining。
     */
    id = MotorHw_FromTim(htim);
    if (!motor_id_valid(id)) {
        return;
    }

    m = &g_motor[id];
    if ((m->mode == MOTOR_MODE_IDLE) || (m->mode == MOTOR_MODE_ESTOP) || (m->applied_freq_hz == 0U)) {
        return;
    }

    /* 只要电机处于有效输出状态，就累计总步数。 */
    m->steps_total++;

    if ((m->mode == MOTOR_MODE_MOVE_STEPS) ||
        (m->mode == MOTOR_MODE_RECIP_FINITE) ||
        (m->mode == MOTOR_MODE_RECIP_INFINITE)) {
        if (m->step_remaining > 0U) {
            m->step_remaining--;
            if (m->step_remaining == 0U) {
                if (m->mode == MOTOR_MODE_MOVE_STEPS) {
                    motor_set_idle(id, m);
                } else {
                    motor_handle_recip_leg_done(id, m);
                }
            }
        }
    }
}

int MotorCtrl_Run(MotorId_t id, MotorDir_t dir, float rpm)
{
    MotorRuntime_t *m;

    if (!motor_id_valid(id)) {
        return -1;
    }

    m = &g_motor[id];
    if (!motor_rpm_valid(m, rpm)) {
        return -2;
    }

    /* 连续运行模式：不限制步数，直到 Stop/EStop 或新命令覆盖。 */
    m->dir = dir;
    m->rpm_target = rpm;
    m->rpm_cur = rpm;
    m->mode = MOTOR_MODE_RUN;
    m->ramp_total_ms = 0U;
    m->step_remaining = 0U;
    return 0;
}

int MotorCtrl_Stop(MotorId_t id, uint32_t soft_stop_ms)
{
    MotorRuntime_t *m;

    if (!motor_id_valid(id)) {
        return -1;
    }

    if ((soft_stop_ms != 0U) &&
        ((soft_stop_ms < MOTOR_SOFT_STOP_MIN_MS) || (soft_stop_ms > MOTOR_SOFT_STOP_MAX_MS))) {
        return -2;
    }

    /* 普通停止走软停止流程，减少机械冲击。 */
    m = &g_motor[id];
    if (m->mode == MOTOR_MODE_IDLE) {
        return 0;
    }
    motor_start_soft_stop(m, soft_stop_ms);
    return 0;
}

int MotorCtrl_EStop(MotorId_t id)
{
    MotorRuntime_t *m;

    if (!motor_id_valid(id)) {
        return -1;
    }

    /* 急停立即关闭 PWM 和使能，不等待软停止曲线。 */
    m = &g_motor[id];
    motor_hw_stop(id, m);
    m->mode = MOTOR_MODE_ESTOP;
    return 0;
}

int MotorCtrl_SetSpeed(MotorId_t id, float rpm)
{
    MotorRuntime_t *m;

    if (!motor_id_valid(id)) {
        return -1;
    }

    m = &g_motor[id];
    if (!motor_rpm_valid(m, rpm)) {
        return -2;
    }

    /* SetSpeed 直接更新目标和当前转速，适合调试命令即时改速。 */
    m->rpm_target = rpm;
    m->rpm_cur = rpm;
    m->mode = MOTOR_MODE_RUN;
    return 0;
}

int MotorCtrl_AccelTo(MotorId_t id, float to_rpm, uint32_t duration_ms)
{
    MotorRuntime_t *m;

    if ((!motor_id_valid(id)) || (duration_ms == 0U)) {
        return -1;
    }

    m = &g_motor[id];
    if (!motor_rpm_valid(m, to_rpm)) {
        return -2;
    }

    /* 在指定时间内线性过渡到目标转速，方向不在这里改变。 */
    m->ramp_start_rpm = m->rpm_cur;
    m->ramp_target_rpm = to_rpm;
    m->ramp_total_ms = duration_ms;
    m->ramp_elapsed_ms = 0U;
    m->rpm_target = to_rpm;
    m->mode = (to_rpm >= m->rpm_cur) ? MOTOR_MODE_RAMP_UP : MOTOR_MODE_RAMP_DOWN;
    return 0;
}

int MotorCtrl_DecelTo(MotorId_t id, float to_rpm, uint32_t duration_ms)
{
    return MotorCtrl_AccelTo(id, to_rpm, duration_ms);
}

int MotorCtrl_MoveSteps(MotorId_t id, MotorDir_t dir, float rpm, uint32_t steps)
{
    MotorRuntime_t *m;

    if ((!motor_id_valid(id)) || (steps == 0U)) {
        return -1;
    }

    m = &g_motor[id];
    if (!motor_rpm_valid(m, rpm)) {
        return -2;
    }

    /* 指定步数运动：定时器中断每来一个脉冲就扣减 step_remaining。 */
    m->dir = dir;
    m->rpm_target = rpm;
    m->rpm_cur = rpm;
    m->step_remaining = steps;
    m->mode = MOTOR_MODE_MOVE_STEPS;
    m->recip_steps = 0U;
    m->recip_cycles = 0U;
    m->recip_cycles_done = 0U;
    m->recip_return_leg = 0U;
    return 0;
}

int MotorCtrl_Recip(MotorId_t id, MotorDir_t dir, float rpm, uint32_t steps, uint32_t cycles)
{
    MotorRuntime_t *m;

    if ((!motor_id_valid(id)) || (steps == 0U) || (cycles == 0U)) {
        return -1;
    }

    m = &g_motor[id];
    if (!motor_rpm_valid(m, rpm)) {
        return -2;
    }

    /* 有限往返：每个 cycle 包含去程和回程，回程完成后累计完成次数。 */
    m->dir = dir;
    m->rpm_target = rpm;
    m->rpm_cur = rpm;
    m->step_remaining = steps;
    m->recip_steps = steps;
    m->recip_cycles = cycles;
    m->recip_cycles_done = 0U;
    m->recip_return_leg = 0U;
    m->mode = MOTOR_MODE_RECIP_FINITE;
    return 0;
}

int MotorCtrl_RecipInfinite(MotorId_t id, MotorDir_t dir, float rpm, uint32_t steps)
{
    MotorRuntime_t *m;

    if ((!motor_id_valid(id)) || (steps == 0U)) {
        return -1;
    }

    m = &g_motor[id];
    if (!motor_rpm_valid(m, rpm)) {
        return -2;
    }

    /* 无限往返：每完成一个单程就反向，不累计结束条件。 */
    m->dir = dir;
    m->rpm_target = rpm;
    m->rpm_cur = rpm;
    m->step_remaining = steps;
    m->recip_steps = steps;
    m->recip_cycles = 0U;
    m->recip_cycles_done = 0U;
    m->recip_return_leg = 0U;
    m->mode = MOTOR_MODE_RECIP_INFINITE;
    return 0;
}

int MotorCtrl_Configure(MotorId_t id,
                        bool set_ppr, uint32_t ppr,
                        bool set_acc, float acc_rpm_s,
                        bool set_dec, float dec_rpm_s,
                        bool set_stop, uint32_t soft_stop_ms)
{
    MotorRuntime_t *m;

    /*
     * 运行参数配置接口：
     * 调用方通过 set_xxx 标志决定要修改哪些字段，未置位的字段保持原值。
     */
    if (!motor_id_valid(id)) {
        return -1;
    }

    if (!(set_ppr || set_acc || set_dec || set_stop)) {
        return -2;
    }

    m = &g_motor[id];

    if (set_ppr) {
        /* ppr 改变会影响 rpm 和 step_hz 换算，因此需要限制到允许范围内。 */
        if ((ppr < MOTOR_PPR_MIN) || (ppr > MOTOR_PPR_MAX)) {
            return -2;
        }
        m->ppr = ppr;
        if (m->rpm_cur > 0.0f) {
            m->rpm_cur = motor_rpm_clamp(m, m->rpm_cur);
        }
        if (m->rpm_target > 0.0f) {
            m->rpm_target = motor_rpm_clamp(m, m->rpm_target);
        }
    }

    if (set_acc) {
        /* 加速度过小会响应很慢，过大可能造成失步，所以限制在配置范围内。 */
        if ((acc_rpm_s < MOTOR_ACC_DEC_MIN_RPM_S) || (acc_rpm_s > MOTOR_ACC_DEC_MAX_RPM_S)) {
            return -2;
        }
        m->acc_rpm_s = acc_rpm_s;
    }

    if (set_dec) {
        /* 减速度使用和加速度相同的允许范围。 */
        if ((dec_rpm_s < MOTOR_ACC_DEC_MIN_RPM_S) || (dec_rpm_s > MOTOR_ACC_DEC_MAX_RPM_S)) {
            return -2;
        }
        m->dec_rpm_s = dec_rpm_s;
    }

    if (set_stop) {
        /* 软停止时间用于普通 Stop 命令，急停不受这个参数影响。 */
        if ((soft_stop_ms < MOTOR_SOFT_STOP_MIN_MS) || (soft_stop_ms > MOTOR_SOFT_STOP_MAX_MS)) {
            return -2;
        }
        m->soft_stop_ms = soft_stop_ms;
    }

    return 0;
}

void MotorCtrl_StopAll(uint32_t soft_stop_ms)
{
    uint32_t i;

    /* 对所有电机发普通停止命令，仍然遵循各自软停止逻辑。 */
    for (i = 0; i < MOTOR_COUNT; i++) {
        (void)MotorCtrl_Stop((MotorId_t)i, soft_stop_ms);
    }
}

void MotorCtrl_EStopAll(void)
{
    uint32_t i;

    /* 对所有电机立即急停，常用于限位保护、故障保护或全局安全动作。 */
    for (i = 0; i < MOTOR_COUNT; i++) {
        (void)MotorCtrl_EStop((MotorId_t)i);
    }
}

int MotorCtrl_GetStatus(MotorId_t id, MotorStatusSnapshot_t *out_status)
{
    uint32_t primask;
    MotorRuntime_t *m;

    if ((!motor_id_valid(id)) || (out_status == 0)) {
        return -1;
    }

    m = &g_motor[id];

    /*
     * 状态字段会在 1ms Tick 和定时器中断中更新。
     * 这里短暂关中断复制快照，避免读取到一半时 steps_total 等字段被中断修改。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    out_status->mode = m->mode;
    out_status->dir = m->dir;
    out_status->ppr = m->ppr;
    out_status->rpm_current = m->rpm_cur;
    out_status->rpm_target = m->rpm_target;
    out_status->acc_rpm_s = m->acc_rpm_s;
    out_status->dec_rpm_s = m->dec_rpm_s;
    out_status->soft_stop_ms = m->soft_stop_ms;
    out_status->steps_total = m->steps_total;
    out_status->steps_remaining = m->step_remaining;
    out_status->recip_cycles = m->recip_cycles;
    out_status->recip_cycles_done = m->recip_cycles_done;
    if (primask == 0U) {
        __enable_irq();
    }

    return 0;
}

const char *MotorCtrl_ModeToString(MotorMode_t mode)
{
    /* 给串口状态命令使用的英文模式文本。 */
    switch (mode) {
        case MOTOR_MODE_IDLE: return "IDLE";
        case MOTOR_MODE_RUN: return "RUN";
        case MOTOR_MODE_RAMP_UP: return "RAMP_UP";
        case MOTOR_MODE_RAMP_DOWN: return "RAMP_DOWN";
        case MOTOR_MODE_MOVE_STEPS: return "MOVE_STEPS";
        case MOTOR_MODE_RECIP_FINITE: return "RECIP_FINITE";
        case MOTOR_MODE_RECIP_INFINITE: return "RECIP_INFINITE";
        case MOTOR_MODE_SOFT_STOP: return "SOFT_STOP";
        case MOTOR_MODE_ESTOP: return "ESTOP";
        default: return "UNKNOWN";
    }
}

const char *MotorCtrl_DirToString(MotorDir_t dir)
{
    /* 给串口状态命令使用的方向文本。 */
    return (dir == MOTOR_DIR_CCW) ? "CCW" : "CW";
}
