#include "axis_planner.h"

#include <math.h>
#include <string.h>
#include "axis_limit_store.h"
#include "motor_ctrl.h"

/*
 * 三轴运动规划运行状态。
 * 本模块只规划 XYZ 三轴同步到目标点，不直接管理 M2 旋转轴和 M5 夹爪。
 *
 * 重要说明：
 * 当前 XYZ 三轴没有 PID 控制器。
 * 本模块是“目标坐标 -> 位移差 -> 按最大位移轴比例分配 STEP 频率 -> MotorCtrl_MoveSteps() 输出脉冲”的开环规划，
 * 编码器/坐标反馈只用于到位容差判断和状态诊断，没有 Kp/Ki/Kd、积分项、微分项或误差闭环调速。
 * 如果后续要做 PID，需要新增位置误差采样、PID 参数、输出限幅和周期调速逻辑，不能只改这里的注释。
 */
typedef struct {
    /* 当前规划状态：空闲、运行、完成或故障。 */
    AxisPlanState_t state;
    /* 运动开始时的坐标和目标坐标，单位都是步数。 */
    AxisPos_t start;
    AxisPos_t target;
    /* 规划参数：默认移动转速、编码器误差阈值、位置完成容差。 */
    AxisPlanConfig_t cfg;
    /* 故障信息，用于状态查询和上位机诊断。 */
    AxisPlanFaultCode_t fault_code;
    AxisId_t fault_axis;
    int32_t fault_value;
    uint32_t fault_ref;
    /* active_mask 标记当前有哪些轴参与本次运动。 */
    uint32_t active_mask;
    /* 各轴运动开始时的电机累计步数，保留给后续扩展或诊断使用。 */
    uint32_t start_steps_total[AXIS_ID_COUNT];
} AxisPlannerRuntime_t;

/* 三轴规划器的全局状态。 */
static AxisPlannerRuntime_t g_axis_plan;

static uint8_t axis_valid(AxisId_t axis)
{
    return (axis < AXIS_ID_COUNT) ? 1U : 0U;
}

static uint32_t axis_abs_i32(int32_t v)
{
    /* 坐标差值取绝对值，用于比较哪个轴位移最大。 */
    return (v >= 0) ? (uint32_t)v : (uint32_t)(-v);
}

static uint32_t axis_plan_rpm_to_step_hz(float rpm, uint32_t ppr)
{
    float hz_f;
    uint32_t hz;

    if ((rpm <= 0.0f) || (ppr == 0U)) {
        return 0U;
    }

    /* rpm 转 step_hz，并限制到电机硬件支持的频率范围内。 */
    hz_f = rpm * (float)ppr / 60.0f;
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

static float axis_plan_step_hz_to_rpm(uint32_t step_hz, uint32_t ppr)
{
    /* step_hz 转 rpm，给每个轴按比例分配速度后再换算回电机控制接口需要的 rpm。 */
    if ((step_hz == 0U) || (ppr == 0U)) {
        return 0.0f;
    }
    return ((float)step_hz * 60.0f) / (float)ppr;
}

static uint32_t axis_plan_cfg_rpm_to_x10(float rpm)
{
    /* Flash 中保存 rpm * 10 的整数，避免直接保存 float。 */
    if (rpm <= 0.0f) {
        return 0U;
    }
    return (uint32_t)(rpm * 10.0f + 0.5f);
}

static float axis_plan_cfg_x10_to_rpm(uint32_t rpm_x10)
{
    return ((float)rpm_x10) / 10.0f;
}

static void axis_plan_set_fault(AxisPlanFaultCode_t code,
                                AxisId_t axis,
                                int32_t value,
                                uint32_t ref)
{
    /* 记录规划器故障时短暂关中断，避免状态查询读到半更新的数据。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_axis_plan.state = AXIS_PLAN_STATE_FAULT;
    g_axis_plan.fault_code = code;
    g_axis_plan.fault_axis = axis;
    g_axis_plan.fault_value = value;
    g_axis_plan.fault_ref = ref;
    if (primask == 0U) {
        __enable_irq();
    }
}

const char *AxisPlanner_StateToString(AxisPlanState_t state)
{
    switch (state) {
        case AXIS_PLAN_STATE_IDLE: return "IDLE";
        case AXIS_PLAN_STATE_RUNNING: return "RUNNING";
        case AXIS_PLAN_STATE_DONE: return "DONE";
        case AXIS_PLAN_STATE_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

const char *AxisPlanner_FaultToString(AxisPlanFaultCode_t fault)
{
    switch (fault) {
        case AXIS_PLAN_FAULT_NONE: return "NONE";
        case AXIS_PLAN_FAULT_LIMIT: return "LIMIT";
        case AXIS_PLAN_FAULT_ENCODER_MISMATCH: return "ENCODER_MISMATCH";
        case AXIS_PLAN_FAULT_MOTOR_CMD: return "MOTOR_CMD";
        default: return "UNKNOWN";
    }
}

void AxisPlanner_Init(void)
{
    uint32_t rpm_x10 = AXIS_PLAN_DEFAULT_RPM_X10;
    uint32_t enc_err = AXIS_PLAN_DEFAULT_ENC_ERR;
    uint32_t pos_tol = AXIS_PLAN_DEFAULT_POS_TOL;

    /* 启动时从 Flash 读取规划参数，读取失败则写入默认值。 */
    memset(&g_axis_plan, 0, sizeof(g_axis_plan));
    g_axis_plan.state = AXIS_PLAN_STATE_IDLE;
    g_axis_plan.fault_code = AXIS_PLAN_FAULT_NONE;
    g_axis_plan.fault_axis = AXIS_ID_INVALID;

    if (AxisLimitStore_LoadPlanner(&rpm_x10, &enc_err, &pos_tol) != 0) {
        (void)AxisLimitStore_SavePlanner(rpm_x10, enc_err, pos_tol);
    }

    g_axis_plan.cfg.move_rpm_default = axis_plan_cfg_x10_to_rpm(rpm_x10);
    g_axis_plan.cfg.enc_err_max = enc_err;
    g_axis_plan.cfg.pos_tol = pos_tol;
}

static int axis_plan_prepare_motor(AxisId_t axis,
                                   int32_t delta,
                                   uint32_t max_delta,
                                   uint32_t base_step_hz,
                                   uint32_t *start_steps_total)
{
    MotorId_t motor;
    MotorStatusSnapshot_t st;
    float rpm_axis;
    float min_rpm;
    float max_rpm;
    uint32_t step_hz_axis;
    MotorDir_t dir;

    /*
     * 为单个坐标轴准备电机运动。
     * max_delta 是本次三轴运动中最大的位移，base_step_hz 属于最大位移轴。
     * 其他轴按 abs(delta) / max_delta 的比例降低 step_hz，从而尽量同时到达目标点。
     * 这里没有按位置误差动态调整速度，因此不是 PID，只是按比例计算一次目标脉冲频率。
     */
    if ((!axis_valid(axis)) || (start_steps_total == 0) || (max_delta == 0U)) {
        return -1;
    }

    motor = AxisCoord_ToMotor(axis);
    if (motor == MOTOR_ID_INVALID) {
        return -1;
    }

    if (MotorCtrl_GetStatus(motor, &st) != 0) {
        return -1;
    }

    /* 用 64 位中间值避免 base_step_hz * delta 在 32 位下溢出。 */
    step_hz_axis = (uint32_t)((((uint64_t)base_step_hz * (uint64_t)axis_abs_i32(delta)) + ((uint64_t)max_delta / 2ULL)) /
                              (uint64_t)max_delta);
    if (step_hz_axis < MOTOR_STEP_FREQ_MIN_HZ) {
        step_hz_axis = MOTOR_STEP_FREQ_MIN_HZ;
    } else if (step_hz_axis > MOTOR_STEP_FREQ_MAX_HZ) {
        step_hz_axis = MOTOR_STEP_FREQ_MAX_HZ;
    }

    rpm_axis = axis_plan_step_hz_to_rpm(step_hz_axis, st.ppr);
    min_rpm = MotorCtrl_GetRpmMin(st.ppr);
    max_rpm = MotorCtrl_GetRpmMax(st.ppr);

    if (rpm_axis < min_rpm) {
        rpm_axis = min_rpm;
    }
    if (rpm_axis > max_rpm) {
        rpm_axis = max_rpm;
    }

    /* delta 正负表示逻辑坐标增减，再按每轴机械方向映射到真实 CW/CCW。 */
    dir = MotorHw_GetDirectionForPositionChange(motor, delta >= 0);
    if (MotorCtrl_MoveSteps(motor, dir, rpm_axis, axis_abs_i32(delta)) != 0) {
        return -2;
    }

    if (MotorCtrl_GetStatus(motor, &st) != 0) {
        return -1;
    }
    *start_steps_total = st.steps_total;
    return 0;
}

int AxisPlanner_MoveTo(int32_t x, int32_t y, int32_t z, float rpm_hint)
{
    AxisPos_t cur;
    int32_t delta[AXIS_ID_COUNT];
    uint32_t abs_delta[AXIS_ID_COUNT];
    uint32_t max_delta = 0U;
    float base_rpm;
    uint32_t base_step_hz;
    AxisId_t dom_axis = AXIS_ID_INVALID;
    MotorId_t dom_motor;
    MotorStatusSnapshot_t dom_st;
    uint32_t i;
    uint32_t active_mask = 0U;
    uint32_t local_start_steps[AXIS_ID_COUNT];

    /*
     * 如果上一段规划还在运行，新目标到来时先急停旧运动并清理状态。
     * 这样可以避免两个规划同时争用同一组电机。
     */
    if (g_axis_plan.state == AXIS_PLAN_STATE_RUNNING) {
        (void)AxisPlanner_EStop();
        g_axis_plan.state = AXIS_PLAN_STATE_IDLE;
        g_axis_plan.active_mask = 0U;
        g_axis_plan.fault_code = AXIS_PLAN_FAULT_NONE;
        g_axis_plan.fault_axis = AXIS_ID_INVALID;
        g_axis_plan.fault_value = 0;
        g_axis_plan.fault_ref = 0U;
    }
    g_axis_plan.active_mask = 0U;

    if (AxisCoord_GetAll(&cur) != 0) {
        return -2;
    }

    /* 计算目标点相对当前位置的三轴位移。 */
    delta[AXIS_ID_X] = x - cur.x;
    delta[AXIS_ID_Y] = y - cur.y;
    delta[AXIS_ID_Z] = z - cur.z;

    /* 找出位移最大的主导轴，用它的速度作为同步运动的基准速度。 */
    for (i = 0U; i < AXIS_ID_COUNT; i++) {
        abs_delta[i] = axis_abs_i32(delta[i]);
        if (abs_delta[i] > max_delta) {
            max_delta = abs_delta[i];
            dom_axis = (AxisId_t)i;
        }
    }

    /* 目标点等于当前位置时，不启动电机，直接标记 DONE。 */
    if (max_delta == 0U) {
        g_axis_plan.state = AXIS_PLAN_STATE_DONE;
        g_axis_plan.target = cur;
        g_axis_plan.active_mask = 0U;
        g_axis_plan.fault_code = AXIS_PLAN_FAULT_NONE;
        return 0;
    }

    /* 外部传入 rpm_hint 时优先使用，否则使用保存的默认规划转速。 */
    base_rpm = (rpm_hint > 0.0f) ? rpm_hint : g_axis_plan.cfg.move_rpm_default;
    if (base_rpm <= 0.0f) {
        base_rpm = axis_plan_cfg_x10_to_rpm(AXIS_PLAN_DEFAULT_RPM_X10);
    }
    if (dom_axis == AXIS_ID_INVALID) {
        return -2;
    }

    dom_motor = AxisCoord_ToMotor(dom_axis);
    if ((dom_motor == MOTOR_ID_INVALID) || (MotorCtrl_GetStatus(dom_motor, &dom_st) != 0)) {
        return -2;
    }
    base_step_hz = axis_plan_rpm_to_step_hz(base_rpm, dom_st.ppr);
    if (base_step_hz == 0U) {
        return -2;
    }

    /*
     * 逐个启动有位移的轴。
     * 任意轴启动失败时立即急停，并记录为 MOTOR_CMD 故障。
     */
    memset(local_start_steps, 0, sizeof(local_start_steps));
    for (i = 0U; i < AXIS_ID_COUNT; i++) {
        if (abs_delta[i] == 0U) {
            continue;
        }

        if (axis_plan_prepare_motor((AxisId_t)i, delta[i], max_delta, base_step_hz, &local_start_steps[i]) != 0) {
            (void)AxisPlanner_EStop();
            axis_plan_set_fault(AXIS_PLAN_FAULT_MOTOR_CMD, (AxisId_t)i, 0, 0U);
            return -4;
        }

        active_mask |= (1UL << i);
        g_axis_plan.active_mask = active_mask;
    }

    /* 所有轴启动成功后，才把规划器状态切换为 RUNNING。 */
    g_axis_plan.start = cur;
    g_axis_plan.target.x = x;
    g_axis_plan.target.y = y;
    g_axis_plan.target.z = z;
    g_axis_plan.active_mask = active_mask;
    memcpy(g_axis_plan.start_steps_total, local_start_steps, sizeof(local_start_steps));
    g_axis_plan.state = AXIS_PLAN_STATE_RUNNING;
    g_axis_plan.fault_code = AXIS_PLAN_FAULT_NONE;
    g_axis_plan.fault_axis = AXIS_ID_INVALID;
    g_axis_plan.fault_value = 0;
    g_axis_plan.fault_ref = 0U;

    return 0;
}

void AxisPlanner_Tick1ms(void)
{
    AxisPos_t cur;
    uint8_t done = 1U;

    /*
     * 规划器 1ms 状态检查：
     * 当前实现通过 AxisCoord 的坐标反馈判断是否进入目标容差范围。
     * 这里只判断“是否到位”，没有根据误差重新计算 PWM/STEP 输出，所以不属于 PID 闭环控制。
     */
    if (g_axis_plan.state != AXIS_PLAN_STATE_RUNNING) {
        return;
    }

    if (AxisCoord_GetAll(&cur) != 0) {
        return;
    }

    /* 三个坐标轴全部进入 pos_tol 容差范围后，认为本次 MoveTo 完成。 */
    if (axis_abs_i32(g_axis_plan.target.x - cur.x) > g_axis_plan.cfg.pos_tol) done = 0U;
    if (axis_abs_i32(g_axis_plan.target.y - cur.y) > g_axis_plan.cfg.pos_tol) done = 0U;
    if (axis_abs_i32(g_axis_plan.target.z - cur.z) > g_axis_plan.cfg.pos_tol) done = 0U;

    if (done != 0U) {
        g_axis_plan.state = AXIS_PLAN_STATE_DONE;
    }
}

int AxisPlanner_Stop(void)
{
    uint32_t i;

    /* 普通停止只停止本次规划中 active_mask 标记的轴。 */
    for (i = 0U; i < AXIS_ID_COUNT; i++) {
        MotorId_t motor;
        if ((g_axis_plan.active_mask & (1UL << i)) == 0U) {
            continue;
        }
        motor = AxisCoord_ToMotor((AxisId_t)i);
        if (motor != MOTOR_ID_INVALID) {
            (void)MotorCtrl_Stop(motor, 0U);
        }
    }

    g_axis_plan.state = AXIS_PLAN_STATE_IDLE;
    g_axis_plan.active_mask = 0U;
    return 0;
}

int AxisPlanner_EStop(void)
{
    uint32_t i;

    /* 急停同样只作用于规划器正在管理的 XYZ 轴，并把规划器置为 FAULT。 */
    for (i = 0U; i < AXIS_ID_COUNT; i++) {
        MotorId_t motor;
        if ((g_axis_plan.active_mask & (1UL << i)) == 0U) {
            continue;
        }
        motor = AxisCoord_ToMotor((AxisId_t)i);
        if (motor != MOTOR_ID_INVALID) {
            (void)MotorCtrl_EStop(motor);
        }
    }

    g_axis_plan.state = AXIS_PLAN_STATE_FAULT;
    g_axis_plan.active_mask = 0U;
    return 0;
}

int AxisPlanner_GetConfig(AxisPlanConfig_t *out)
{
    /* 返回当前规划参数。 */
    if (out == 0) {
        return -1;
    }
    *out = g_axis_plan.cfg;
    return 0;
}

int AxisPlanner_SetAndSaveConfig(float move_rpm_default, uint32_t enc_err_max, uint32_t pos_tol)
{
    uint32_t rpm_x10;

    /* 参数先做范围校验，再写入 Flash，最后更新 RAM 中的运行配置。 */
    rpm_x10 = axis_plan_cfg_rpm_to_x10(move_rpm_default);
    if ((rpm_x10 < AXIS_PLAN_RPM_MIN_X10) || (rpm_x10 > AXIS_PLAN_RPM_MAX_X10)) {
        return -1;
    }
    if ((enc_err_max < AXIS_PLAN_ERR_MIN) || (enc_err_max > AXIS_PLAN_ERR_MAX)) {
        return -1;
    }
    if ((pos_tol < AXIS_PLAN_POS_TOL_MIN) || (pos_tol > AXIS_PLAN_POS_TOL_MAX)) {
        return -1;
    }

    if (AxisLimitStore_SavePlanner(rpm_x10, enc_err_max, pos_tol) != 0) {
        return -2;
    }

    g_axis_plan.cfg.move_rpm_default = move_rpm_default;
    g_axis_plan.cfg.enc_err_max = enc_err_max;
    g_axis_plan.cfg.pos_tol = pos_tol;
    return 0;
}

int AxisPlanner_GetStatus(AxisPlanStatus_t *out)
{
    AxisPos_t cur;

    /* 汇总当前坐标、目标坐标、状态、故障和配置，供串口状态命令使用。 */
    if (out == 0) {
        return -1;
    }

    if (AxisCoord_GetAll(&cur) != 0) {
        memset(&cur, 0, sizeof(cur));
    }

    out->state = g_axis_plan.state;
    out->current = cur;
    out->target = g_axis_plan.target;
    out->fault_code = g_axis_plan.fault_code;
    out->fault_axis = g_axis_plan.fault_axis;
    out->fault_value = g_axis_plan.fault_value;
    out->fault_ref = g_axis_plan.fault_ref;
    out->cfg = g_axis_plan.cfg;
    return 0;
}
