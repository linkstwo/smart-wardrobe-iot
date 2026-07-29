#include "axis_coord.h"

#include <limits.h>
#include <string.h>
#include "axis_limit_store.h"
#include "motor_ctrl.h"

/*
 * 三轴坐标模块运行状态：
 * X/Y/Z 使用 M1/M3/M4 的 steps_total 增量累计位置；
 * M2 是旋转轴/绝对值编码器相关轴，不属于 XYZ 三轴坐标，因此单独保存 m3_rot_steps。
 */
typedef struct {
    /* XYZ 当前位置，单位为步数。 */
    int32_t pos[AXIS_ID_COUNT];
    /* 上一次读取到的各轴电机累计步数，用于计算本次增量。 */
    uint32_t prev_steps_total[AXIS_ID_COUNT];
    /* M2 旋转轴累计步数和上次读取的累计步数。 */
    int32_t m3_rot_steps;
    uint32_t m3_prev_steps_total;
} AxisCoordRuntime_t;

/* 坐标模块的全局运行状态。 */
static AxisCoordRuntime_t g_axis_coord;

static uint8_t axis_valid(AxisId_t axis)
{
    return (axis < AXIS_ID_COUNT) ? 1U : 0U;
}

static int32_t axis_add_sat_i32(int32_t base, int32_t delta)
{
    int64_t sum = (int64_t)base + (int64_t)delta;

    /* 使用饱和加法，防止长时间运行后位置累计溢出 int32_t。 */
    if (sum > (int64_t)INT32_MAX) {
        return INT32_MAX;
    }
    if (sum < (int64_t)INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)sum;
}

static float steps_to_deg(int32_t steps, uint32_t ppr)
{
    /* 步数转角度：steps / ppr * 360。 */
    if (ppr == 0U) {
        return 0.0f;
    }
    return ((float)steps * 360.0f) / (float)ppr;
}

static int32_t deg_to_steps(float deg, uint32_t ppr)
{
    float steps_f;
    int64_t steps_i64;

    /* 角度转步数，并按正负方向分别四舍五入。 */
    if (ppr == 0U) {
        return 0;
    }

    steps_f = deg * (float)ppr / 360.0f;
    if (steps_f >= 0.0f) {
        steps_i64 = (int64_t)(steps_f + 0.5f);
    } else {
        steps_i64 = (int64_t)(steps_f - 0.5f);
    }

    if (steps_i64 > (int64_t)INT32_MAX) {
        return INT32_MAX;
    }
    if (steps_i64 < (int64_t)INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)steps_i64;
}

AxisId_t AxisCoord_FromMotor(MotorId_t motor_id)
{
    /*
     * 衣柜三轴坐标和电机编号的固定映射：
     * X 轴 = M1；
     * Y 轴 = M3；
     * Z 轴 = M4；
     * M2 为旋转轴，不映射到 XYZ；
     * M5 为夹爪，不映射到 XYZ。
     */
    switch (motor_id) {
        case MOTOR_ID_1: return AXIS_ID_X;
        case MOTOR_ID_3: return AXIS_ID_Y;
        case MOTOR_ID_4: return AXIS_ID_Z;
        default: return AXIS_ID_INVALID;
    }
}

MotorId_t AxisCoord_ToMotor(AxisId_t axis)
{
    /* 从坐标轴反查对应电机，三轴联动规划会使用这个映射启动电机。 */
    switch (axis) {
        case AXIS_ID_X: return MOTOR_ID_1;
        case AXIS_ID_Y: return MOTOR_ID_3;
        case AXIS_ID_Z: return MOTOR_ID_4;
        default: return MOTOR_ID_INVALID;
    }
}

void AxisCoord_Init(void)
{
    uint32_t i;
    MotorStatusSnapshot_t st_rot;
    float saved_deg = 0.0f;

    /*
     * 初始化时先读取各电机当前累计步数作为基准。
     * 后续 Tick 只累加“当前 steps_total - 上一次 steps_total”的增量。
     */
    memset(&g_axis_coord, 0, sizeof(g_axis_coord));

    for (i = 0U; i < AXIS_ID_COUNT; i++) {
        MotorId_t motor = AxisCoord_ToMotor((AxisId_t)i);
        MotorStatusSnapshot_t st;

        if ((motor != MOTOR_ID_INVALID) && (MotorCtrl_GetStatus(motor, &st) == 0)) {
            g_axis_coord.prev_steps_total[i] = st.steps_total;
        } else {
            g_axis_coord.prev_steps_total[i] = 0U;
        }
        g_axis_coord.pos[i] = 0;
    }

    /*
     * M2 旋转轴的角度可以从 Flash 中恢复。
     * 如果 Flash 中没有保存角度，则以 0 作为当前旋转角基准。
     */
    if (MotorCtrl_GetStatus(AXIS_ROT_MOTOR_ID, &st_rot) == 0) {
        g_axis_coord.m3_prev_steps_total = st_rot.steps_total;
        if (AxisCoord_M3RotLoadSavedFromFlash(&saved_deg) == 0) {
            g_axis_coord.m3_rot_steps = deg_to_steps(saved_deg, st_rot.ppr);
        } else {
            g_axis_coord.m3_rot_steps = 0;
        }
    } else {
        g_axis_coord.m3_prev_steps_total = 0U;
        g_axis_coord.m3_rot_steps = 0;
    }
}

void AxisCoord_Tick1ms(void)
{
    uint32_t i;

    /*
     * 每 1ms 根据电机累计步数刷新 XYZ 坐标。
     * MotorCtrl_OnTimUpdate() 负责累计 steps_total，本函数只消费这个累计量。
     */
    for (i = 0U; i < AXIS_ID_COUNT; i++) {
        MotorId_t motor = AxisCoord_ToMotor((AxisId_t)i);
        MotorStatusSnapshot_t st;
        uint32_t now_steps;
        uint32_t prev_steps;
        uint32_t delta_u32;
        int32_t delta;

        if (motor == MOTOR_ID_INVALID) {
            continue;
        }
        if (MotorCtrl_GetStatus(motor, &st) != 0) {
            continue;
        }

        /* 使用无符号差值计算增量，适配 steps_total 正常递增的情况。 */
        now_steps = st.steps_total;
        prev_steps = g_axis_coord.prev_steps_total[i];
        delta_u32 = now_steps - prev_steps;
        g_axis_coord.prev_steps_total[i] = now_steps;

        if (delta_u32 == 0U) {
            continue;
        }

        if (delta_u32 > (uint32_t)INT32_MAX) {
            delta_u32 = (uint32_t)INT32_MAX;
        }
        delta = (int32_t)delta_u32;
        /* 实机坐标正方向统一为 CW；M4 的 CW 对应向下。 */
        if (!MotorHw_DirectionIncreasesPosition(motor, st.dir)) {
            delta = -delta;
        }

        g_axis_coord.pos[i] = axis_add_sat_i32(g_axis_coord.pos[i], delta);
    }

    {
        /*
         * M2 旋转轴不参与 XYZ 三轴坐标，但仍需要累计旋转步数，
         * 用于旋转角度显示、保存和复位。
         */
        MotorStatusSnapshot_t st_rot;
        uint32_t now_steps;
        uint32_t prev_steps;
        uint32_t delta_u32;
        int32_t delta;

        if (MotorCtrl_GetStatus(AXIS_ROT_MOTOR_ID, &st_rot) != 0) {
            return;
        }

        now_steps = st_rot.steps_total;
        prev_steps = g_axis_coord.m3_prev_steps_total;
        delta_u32 = now_steps - prev_steps;
        g_axis_coord.m3_prev_steps_total = now_steps;

        if (delta_u32 == 0U) {
            return;
        }

        if (delta_u32 > (uint32_t)INT32_MAX) {
            delta_u32 = (uint32_t)INT32_MAX;
        }
        delta = (int32_t)delta_u32;
        if (st_rot.dir == MOTOR_DIR_CCW) {
            delta = -delta;
        }

        g_axis_coord.m3_rot_steps = axis_add_sat_i32(g_axis_coord.m3_rot_steps, delta);
    }
}

int AxisCoord_GetPos(AxisId_t axis, int32_t *out_steps)
{
    uint32_t primask;

    /* 单轴位置读取，短暂关中断保证读取的是一致的 int32_t 值。 */
    if ((!axis_valid(axis)) || (out_steps == 0)) {
        return -1;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out_steps = g_axis_coord.pos[axis];
    if (primask == 0U) {
        __enable_irq();
    }

    return 0;
}

int AxisCoord_GetAll(AxisPos_t *out)
{
    uint32_t primask;

    /* 一次性读取 XYZ 三轴位置，避免上位机状态查询时三轴数据来自不同瞬间。 */
    if (out == 0) {
        return -1;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    out->x = g_axis_coord.pos[AXIS_ID_X];
    out->y = g_axis_coord.pos[AXIS_ID_Y];
    out->z = g_axis_coord.pos[AXIS_ID_Z];
    if (primask == 0U) {
        __enable_irq();
    }

    return 0;
}

int AxisCoord_ZeroNow(void)
{
    uint32_t i;
    uint32_t latest_steps[AXIS_ID_COUNT];
    uint32_t primask;

    /*
     * 当前点设为三轴坐标原点。
     * 先读取最新 steps_total，再在关中断区间内同时清坐标和更新基准。
     */
    for (i = 0U; i < AXIS_ID_COUNT; i++) {
        MotorId_t motor = AxisCoord_ToMotor((AxisId_t)i);
        MotorStatusSnapshot_t st;

        if (motor == MOTOR_ID_INVALID) {
            return -1;
        }
        if (MotorCtrl_GetStatus(motor, &st) != 0) {
            return -1;
        }
        latest_steps[i] = st.steps_total;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    g_axis_coord.pos[AXIS_ID_X] = 0;
    g_axis_coord.pos[AXIS_ID_Y] = 0;
    g_axis_coord.pos[AXIS_ID_Z] = 0;
    for (i = 0U; i < AXIS_ID_COUNT; i++) {
        g_axis_coord.prev_steps_total[i] = latest_steps[i];
    }
    if (primask == 0U) {
        __enable_irq();
    }

    return 0;
}

int AxisCoord_M3RotGet(int32_t *out_steps, float *out_deg)
{
    uint32_t primask;
    MotorStatusSnapshot_t st_rot;
    int32_t steps;

    /* 读取 M2 旋转轴累计步数，并按当前 ppr 换算成角度。 */
    if ((out_steps == 0) || (out_deg == 0)) {
        return -1;
    }
    if (MotorCtrl_GetStatus(AXIS_ROT_MOTOR_ID, &st_rot) != 0) {
        return -2;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    steps = g_axis_coord.m3_rot_steps;
    if (primask == 0U) {
        __enable_irq();
    }

    *out_steps = steps;
    *out_deg = steps_to_deg(steps, st_rot.ppr);
    return 0;
}

int AxisCoord_M3RotZeroNow(void)
{
    MotorStatusSnapshot_t st_rot;
    uint32_t primask;

    /* 将当前 M2 旋转位置作为 0 点，只修改软件累计基准，不直接驱动电机。 */
    if (MotorCtrl_GetStatus(AXIS_ROT_MOTOR_ID, &st_rot) != 0) {
        return -1;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    g_axis_coord.m3_rot_steps = 0;
    g_axis_coord.m3_prev_steps_total = st_rot.steps_total;
    if (primask == 0U) {
        __enable_irq();
    }

    return 0;
}

int AxisCoord_M3RotSetAndSaveSteps(int32_t steps)
{
    MotorStatusSnapshot_t st_rot;
    uint32_t primask;
    float deg;
    float scaled;
    int32_t deg_x100;

    /*
     * 手动设置 M2 旋转轴软件位置，并转换成角度 * 100 保存到 Flash。
     * 保存为整数可以避免 Flash 数据格式依赖 float 二进制表示。
     */
    if (MotorCtrl_GetStatus(AXIS_ROT_MOTOR_ID, &st_rot) != 0) {
        return -1;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    g_axis_coord.m3_rot_steps = steps;
    g_axis_coord.m3_prev_steps_total = st_rot.steps_total;
    if (primask == 0U) {
        __enable_irq();
    }

    deg = steps_to_deg(steps, st_rot.ppr);
    scaled = deg * 100.0f;
    if (scaled >= 0.0f) {
        deg_x100 = (int32_t)(scaled + 0.5f);
    } else {
        deg_x100 = (int32_t)(scaled - 0.5f);
    }

    if (AxisLimitStore_SaveM3Angle(deg_x100) != 0) {
        return -2;
    }

    return 0;
}

int AxisCoord_M3RotSaveCurrentToFlash(void)
{
    int32_t steps = 0;
    float deg = 0.0f;
    float scaled;
    int32_t deg_x100;

    /* 把当前 M2 旋转角度保存到 Flash，供下次上电恢复。 */
    if (AxisCoord_M3RotGet(&steps, &deg) != 0) {
        return -1;
    }

    scaled = deg * 100.0f;
    if (scaled >= 0.0f) {
        deg_x100 = (int32_t)(scaled + 0.5f);
    } else {
        deg_x100 = (int32_t)(scaled - 0.5f);
    }

    if (AxisLimitStore_SaveM3Angle(deg_x100) != 0) {
        return -2;
    }
    return 0;
}

int AxisCoord_M3RotLoadSavedFromFlash(float *out_deg)
{
    int32_t deg_x100;

    /* 从 Flash 读取角度 * 100，再还原成 float 角度值。 */
    if (out_deg == 0) {
        return -1;
    }
    if (AxisLimitStore_LoadM3Angle(&deg_x100) != 0) {
        return -2;
    }

    *out_deg = ((float)deg_x100) / 100.0f;
    return 0;
}
