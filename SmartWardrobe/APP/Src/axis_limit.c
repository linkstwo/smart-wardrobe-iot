#include "axis_limit.h"

#include <string.h>
#include "axis_limit_store.h"
#include "motor_ctrl.h"

/* M2 旋转轴的限位故障信息，结构上和 XYZ 轴限位类似，但不绑定 AxisId_t。 */
typedef struct {
    bool active;
    AxisLimitBound_t bound;
    int32_t value;
    uint32_t max;
} AxisRotLimitFault_t;

/*
 * 限位保护模块运行状态：
 * cfg 保存 XYZ 最大行程；
 * m3_rot_max 保存 M2 旋转轴最大步数；
 * protect_enabled 是总保护开关，关闭时只保存参数但不主动急停。
 */
typedef struct {
    AxisLimitConfig_t cfg;
    AxisLimitFault_t fault;
    uint32_t m3_rot_max;
    bool m3_rot_valid;
    AxisRotLimitFault_t m3_rot_fault;
    bool protect_enabled;
} AxisLimitRuntime_t;

static AxisLimitRuntime_t g_axis_limit;

static uint8_t axis_valid(AxisId_t axis)
{
    return (axis < AXIS_ID_COUNT) ? 1U : 0U;
}

static uint32_t axis_get_max(AxisId_t axis)
{
    /* 根据轴编号返回对应最大允许坐标，单位为步数。 */
    switch (axis) {
        case AXIS_ID_X: return g_axis_limit.cfg.x_max;
        case AXIS_ID_Y: return g_axis_limit.cfg.y_max;
        case AXIS_ID_Z: return g_axis_limit.cfg.z_max;
        default: return 0U;
    }
}

static int axis_check_now(AxisId_t axis, AxisLimitFault_t *fault_out)
{
    int32_t pos = 0;
    uint32_t max;

    /*
     * 检查单个 XYZ 轴是否越界：
     * 低边界：pos < 0；
     * 高边界：pos > max。
     */
    if ((!axis_valid(axis)) || (fault_out == 0)) {
        return -1;
    }
    if (!g_axis_limit.cfg.valid) {
        return -2;
    }

    if (AxisCoord_GetPos(axis, &pos) != 0) {
        return -3;
    }

    max = axis_get_max(axis);
    if (pos < 0) {
        fault_out->active = true;
        fault_out->axis = axis;
        fault_out->bound = AXIS_LIMIT_BOUND_LOW;
        fault_out->value = pos;
        fault_out->max = max;
        return 1;
    }

    if ((uint32_t)pos > max) {
        fault_out->active = true;
        fault_out->axis = axis;
        fault_out->bound = AXIS_LIMIT_BOUND_HIGH;
        fault_out->value = pos;
        fault_out->max = max;
        return 1;
    }

    return 0;
}

static int m3_rot_check_now(AxisRotLimitFault_t *fault_out)
{
    int32_t steps = 0;
    float deg_dummy = 0.0f;
    uint32_t max;

    /* M2 旋转轴单独检查 0 ~ m3_rot_max 范围，不参与 XYZ 坐标限制。 */
    if (fault_out == 0) {
        return -1;
    }
    if (!g_axis_limit.m3_rot_valid) {
        return -2;
    }

    if (AxisCoord_M3RotGet(&steps, &deg_dummy) != 0) {
        return -3;
    }

    max = g_axis_limit.m3_rot_max;
    if (steps < 0) {
        fault_out->active = true;
        fault_out->bound = AXIS_LIMIT_BOUND_LOW;
        fault_out->value = steps;
        fault_out->max = max;
        return 1;
    }

    if ((uint32_t)steps > max) {
        fault_out->active = true;
        fault_out->bound = AXIS_LIMIT_BOUND_HIGH;
        fault_out->value = steps;
        fault_out->max = max;
        return 1;
    }

    return 0;
}

static void axis_latch_and_estop(const AxisLimitFault_t *fault)
{
    MotorId_t motor;
    uint32_t primask;

    /* XYZ 任意轴越界时，先急停对应电机，再锁存故障信息。 */
    if ((fault == 0) || !fault->active) {
        return;
    }

    motor = AxisCoord_ToMotor(fault->axis);
    if (motor != MOTOR_ID_INVALID) {
        (void)MotorCtrl_EStop(motor);
    }

    primask = __get_PRIMASK();
    __disable_irq();
    g_axis_limit.fault = *fault;
    if (primask == 0U) {
        __enable_irq();
    }
}

static void m3_rot_latch_and_estop(const AxisRotLimitFault_t *fault)
{
    uint32_t primask;

    /* M2 旋转轴越界时，只急停 M2，并锁存旋转轴故障。 */
    if ((fault == 0) || !fault->active) {
        return;
    }

    (void)MotorCtrl_EStop(AXIS_ROT_MOTOR_ID);

    primask = __get_PRIMASK();
    __disable_irq();
    g_axis_limit.m3_rot_fault = *fault;
    if (primask == 0U) {
        __enable_irq();
    }
}

const char *AxisLimit_AxisToString(AxisId_t axis)
{
    switch (axis) {
        case AXIS_ID_X: return "X";
        case AXIS_ID_Y: return "Y";
        case AXIS_ID_Z: return "Z";
        default: return "UNKNOWN";
    }
}

const char *AxisLimit_BoundToString(AxisLimitBound_t bound)
{
    switch (bound) {
        case AXIS_LIMIT_BOUND_LOW: return "LOW";
        case AXIS_LIMIT_BOUND_HIGH: return "HIGH";
        default: return "NONE";
    }
}

void AxisLimit_Init(void)
{
    uint32_t x_max = 0U;
    uint32_t y_max = 0U;
    uint32_t z_max = 0U;
    uint32_t m3_rot_max = 0U;
    bool protect_enabled = false;
    bool migrated_legacy = false;

    /*
     * 启动时从 Flash 读取限位参数。
     * 如果读取到旧版本数据，AxisLimitStore_LoadAll() 会告诉本函数是否需要迁移写回。
     */
    memset(&g_axis_limit, 0, sizeof(g_axis_limit));

    if (AxisLimitStore_LoadAll(&x_max, &y_max, &z_max, &protect_enabled, &migrated_legacy) == 0) {
        g_axis_limit.cfg.x_max = x_max;
        g_axis_limit.cfg.y_max = y_max;
        g_axis_limit.cfg.z_max = z_max;
        g_axis_limit.cfg.valid = ((x_max > 0U) && (y_max > 0U) && (z_max > 0U)) ? true : false;
        g_axis_limit.protect_enabled = g_axis_limit.cfg.valid ? protect_enabled : false;

        /* 一次性把旧版本数据写回当前版本，补齐新字段。 */
        if (migrated_legacy) {
            (void)AxisLimitStore_SaveAll(x_max, y_max, z_max, g_axis_limit.protect_enabled);
        }
    } else {
        g_axis_limit.cfg.valid = false;
        g_axis_limit.protect_enabled = false;
    }

    /* M2 旋转轴最大步数单独持久化，读取失败则认为该保护项未配置。 */
    if (AxisLimitStore_LoadM3RotMax(&m3_rot_max) == 0) {
        g_axis_limit.m3_rot_max = m3_rot_max;
        g_axis_limit.m3_rot_valid = (m3_rot_max > 0U);
    } else {
        g_axis_limit.m3_rot_max = 0U;
        g_axis_limit.m3_rot_valid = false;
    }

    /* 保护开关只有在至少有一类有效限位参数时才真正开启。 */
    if (protect_enabled) {
        g_axis_limit.protect_enabled = (g_axis_limit.cfg.valid || g_axis_limit.m3_rot_valid);
    } else {
        g_axis_limit.protect_enabled = false;
    }
}

void AxisLimit_Tick1ms(void)
{
    uint32_t i;
    AxisLimitFault_t f;
    AxisRotLimitFault_t rot_f;

    /*
     * 1ms 安全巡检：
     * 保护关闭时不检查；
     * XYZ 限位有效时检查三轴；
     * M2 旋转限位有效时再检查旋转轴。
     */
    if (!g_axis_limit.protect_enabled) {
        return;
    }

    if (g_axis_limit.cfg.valid) {
        memset(&f, 0, sizeof(f));
        for (i = 0U; i < AXIS_ID_COUNT; i++) {
            if (axis_check_now((AxisId_t)i, &f) == 1) {
                axis_latch_and_estop(&f);
                return;
            }
        }
    }

    if (g_axis_limit.m3_rot_valid) {
        memset(&rot_f, 0, sizeof(rot_f));
        if (m3_rot_check_now(&rot_f) == 1) {
            m3_rot_latch_and_estop(&rot_f);
            return;
        }
    }
}

int AxisLimit_SetAndSave(uint32_t x_max, uint32_t y_max, uint32_t z_max)
{
    uint32_t primask;

    /* 设置 XYZ 最大行程并保存到 Flash，三个最大值都必须大于 0。 */
    if ((x_max == 0U) || (y_max == 0U) || (z_max == 0U)) {
        return -1;
    }

    if (AxisLimitStore_SaveAll(x_max, y_max, z_max, g_axis_limit.protect_enabled) != 0) {
        return -2;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    g_axis_limit.cfg.x_max = x_max;
    g_axis_limit.cfg.y_max = y_max;
    g_axis_limit.cfg.z_max = z_max;
    g_axis_limit.cfg.valid = true;
    g_axis_limit.fault.active = false;
    g_axis_limit.fault.axis = AXIS_ID_INVALID;
    g_axis_limit.fault.bound = AXIS_LIMIT_BOUND_NONE;
    g_axis_limit.fault.value = 0;
    g_axis_limit.fault.max = 0U;
    g_axis_limit.m3_rot_fault.active = false;
    g_axis_limit.m3_rot_fault.bound = AXIS_LIMIT_BOUND_NONE;
    g_axis_limit.m3_rot_fault.value = 0;
    g_axis_limit.m3_rot_fault.max = 0U;
    if (primask == 0U) {
        __enable_irq();
    }

    return 0;
}

int AxisLimit_Get(AxisLimitConfig_t *out)
{
    uint32_t primask;

    /* 获取 XYZ 限位配置快照，短暂关中断保证数据一致。 */
    if (out == 0) {
        return -1;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out = g_axis_limit.cfg;
    if (primask == 0U) {
        __enable_irq();
    }

    return 0;
}

int AxisLimit_GetFault(AxisLimitFault_t *out)
{
    uint32_t primask;

    /* 获取 XYZ 限位故障快照。 */
    if (out == 0) {
        return -1;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out = g_axis_limit.fault;
    if (primask == 0U) {
        __enable_irq();
    }

    return 0;
}

int AxisLimit_SetM3RotMaxAndSave(uint32_t rot_max_step)
{
    uint32_t primask;

    /* 设置 M2 旋转轴最大允许步数并保存到 Flash。 */
    if (rot_max_step == 0U) {
        return -1;
    }

    if (AxisLimitStore_SaveM3RotMax(rot_max_step) != 0) {
        return -2;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    g_axis_limit.m3_rot_max = rot_max_step;
    g_axis_limit.m3_rot_valid = true;
    g_axis_limit.m3_rot_fault.active = false;
    g_axis_limit.m3_rot_fault.bound = AXIS_LIMIT_BOUND_NONE;
    g_axis_limit.m3_rot_fault.value = 0;
    g_axis_limit.m3_rot_fault.max = 0U;
    if (primask == 0U) {
        __enable_irq();
    }

    return 0;
}

int AxisLimit_GetM3RotStatus(AxisRotLimitStatus_t *out)
{
    uint32_t primask;

    /* 查询 M2 旋转轴限位配置和当前锁存故障。 */
    if (out == 0) {
        return -1;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    out->valid = g_axis_limit.m3_rot_valid;
    out->max_step = g_axis_limit.m3_rot_max;
    out->fault_active = g_axis_limit.m3_rot_fault.active;
    out->fault_bound = g_axis_limit.m3_rot_fault.bound;
    out->fault_value = g_axis_limit.m3_rot_fault.value;
    if (primask == 0U) {
        __enable_irq();
    }

    return 0;
}

bool AxisLimit_IsReady(void)
{
    /* 保护关闭时认为系统可运行；保护开启时必须先有有效 XYZ 限位参数。 */
    if (!g_axis_limit.protect_enabled) {
        return true;
    }

    return g_axis_limit.cfg.valid;
}

void AxisLimit_ClearFault(void)
{
    uint32_t primask;

    /* 清除 XYZ 和 M2 旋转轴的锁存故障，不修改已保存的限位参数。 */
    primask = __get_PRIMASK();
    __disable_irq();
    g_axis_limit.fault.active = false;
    g_axis_limit.fault.axis = AXIS_ID_INVALID;
    g_axis_limit.fault.bound = AXIS_LIMIT_BOUND_NONE;
    g_axis_limit.fault.value = 0;
    g_axis_limit.fault.max = 0U;
    g_axis_limit.m3_rot_fault.active = false;
    g_axis_limit.m3_rot_fault.bound = AXIS_LIMIT_BOUND_NONE;
    g_axis_limit.m3_rot_fault.value = 0;
    g_axis_limit.m3_rot_fault.max = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

int AxisLimit_SetProtectAndSave(bool enable)
{
    uint32_t primask;

    /* 修改总保护开关并保存，关闭保护时同时清除当前锁存故障。 */
    if (AxisLimitStore_SaveAll(g_axis_limit.cfg.x_max,
                               g_axis_limit.cfg.y_max,
                               g_axis_limit.cfg.z_max,
                               enable) != 0) {
        return -1;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    g_axis_limit.protect_enabled = enable;
    if (!enable) {
        g_axis_limit.fault.active = false;
        g_axis_limit.fault.axis = AXIS_ID_INVALID;
        g_axis_limit.fault.bound = AXIS_LIMIT_BOUND_NONE;
        g_axis_limit.fault.value = 0;
        g_axis_limit.fault.max = 0U;
        g_axis_limit.m3_rot_fault.active = false;
        g_axis_limit.m3_rot_fault.bound = AXIS_LIMIT_BOUND_NONE;
        g_axis_limit.m3_rot_fault.value = 0;
        g_axis_limit.m3_rot_fault.max = 0U;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    return 0;
}

bool AxisLimit_IsProtectEnabled(void)
{
    /* 返回限位保护总开关状态。 */
    return g_axis_limit.protect_enabled;
}

int AxisLimit_CanMoveMotor(MotorId_t motor_id)
{
    AxisId_t axis;
    AxisLimitFault_t f;
    AxisRotLimitFault_t rot_f;
    int rc;

    /*
     * 电机动作前的即时安全检查：
     * - 保护关闭：允许运动；
     * - M2：检查旋转轴限位；
     * - M1/M3/M4：映射到 XYZ 后检查对应坐标轴；
     * - M5 夹爪不属于 XYZ 和 M2 旋转轴，默认允许。
     */
    if (!g_axis_limit.protect_enabled) {
        return 0;
    }

    if (motor_id == AXIS_ROT_MOTOR_ID) {
        if (!g_axis_limit.m3_rot_valid) {
            return -1;
        }

        if (g_axis_limit.m3_rot_fault.active) {
            return -2;
        }

        memset(&rot_f, 0, sizeof(rot_f));
        rc = m3_rot_check_now(&rot_f);
        if (rc == 1) {
            m3_rot_latch_and_estop(&rot_f);
            return -2;
        }
        if (rc < 0) {
            return -3;
        }

        return 0;
    }

    axis = AxisCoord_FromMotor(motor_id);
    if (axis == AXIS_ID_INVALID) {
        return 0;
    }

    if (!g_axis_limit.cfg.valid) {
        return -1;
    }

    if ((g_axis_limit.fault.active) && (g_axis_limit.fault.axis == axis)) {
        return -2;
    }

    memset(&f, 0, sizeof(f));
    rc = axis_check_now(axis, &f);
    if (rc == 1) {
        axis_latch_and_estop(&f);
        return -2;
    }
    if (rc < 0) {
        return -3;
    }

    return 0;
}

void AxisLimit_SetProtectionEnabled(bool enable)
{
    /* 仅修改 RAM 中保护开关，不写 Flash；兼容旧调用路径。 */
    g_axis_limit.protect_enabled = enable;
}

bool AxisLimit_IsProtectionEnabled(void)
{
    /* 旧接口名称的兼容封装。 */
    return AxisLimit_IsProtectEnabled();
}
