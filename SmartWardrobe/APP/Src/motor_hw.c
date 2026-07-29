#include "motor_hw.h"

/*
 * 电机硬件抽象层：
 * 1. 把 M1~M5 映射到具体定时器 PWM 通道、方向 GPIO 和使能 GPIO；
 * 2. 把不同定时器的计数频率统一到 1MHz，方便上层按 step_hz 计算脉冲周期；
 * 3. 这里只负责硬件输出，不处理加减速、步数状态机和衣柜取放业务。
 */
#define TIMX_APB2_TIM_CLK_HZ 168000000UL
#define TIMX_APB1_TIM_CLK_HZ 84000000UL

/*
 * 五个步进电机的硬件映射表。
 * 数组下标必须和 MotorId_t 中的 MOTOR_ID_1~MOTOR_ID_5 保持一致：
 * M1: TIM8  CH1
 * M2: TIM9  CH1
 * M3: TIM10 CH1
 * M4: TIM11 CH1
 * M5: TIM12 CH1
 */
static const MotorHwConfig_t g_motor_cfg[MOTOR_COUNT] = {
    {&htim8, TIM_CHANNEL_1, TIMX_APB2_TIM_CLK_HZ, E1_DIR_GPIO_Port, E1_DIR_Pin, E1_ENABLE_GPIO_Port, E1_ENABLE_Pin},
    {&htim9, TIM_CHANNEL_1, TIMX_APB2_TIM_CLK_HZ, E2_DIR_GPIO_Port, E2_DIR_Pin, E2_ENABLE_GPIO_Port, E2_ENABLE_Pin},
    {&htim10, TIM_CHANNEL_1, TIMX_APB2_TIM_CLK_HZ, E3_DIR_GPIO_Port, E3_DIR_Pin, E3_ENABLE_GPIO_Port, E3_ENABLE_Pin},
    {&htim11, TIM_CHANNEL_1, TIMX_APB2_TIM_CLK_HZ, E4_DIR_GPIO_Port, E4_DIR_Pin, E4_ENABLE_GPIO_Port, E4_ENABLE_Pin},
    {&htim12, TIM_CHANNEL_1, TIMX_APB1_TIM_CLK_HZ, E5_DIR_GPIO_Port, E5_DIR_Pin, E5_ENABLE_GPIO_Port, E5_ENABLE_Pin}
};

/*
 * 每个电机驱动器的使能电平极性：
 * 1：低电平使能；
 * 0：高电平使能。
 *
 * 当前 M1~M5 全部配置为低电平使能。
 * M5 虽然用于夹爪/末端执行机构，但仍然走和 M1~M4 相同的 step/dir/enable 控制链路。
 */
static const uint8_t g_enable_active_low[MOTOR_COUNT] = {
    1U, /* M1 */
    1U, /* M2 */
    1U, /* M3 */
    1U, /* M4 */
    1U  /* M5 */
};

/* 记录每个电机的 PWM 是否已经启动，避免重复打开同一个定时器输出。 */
static uint8_t g_pwm_started[MOTOR_COUNT] = {0};

/* 检查电机编号是否落在 M1~M5 范围内，所有对外接口都会先做这个防御检查。 */
static bool motor_id_valid(MotorId_t id)
{
    return (id < MOTOR_COUNT);
}

static void motor_set_counter_to_1mhz(const MotorHwConfig_t *cfg)
{
    uint32_t psc;

    /* 空配置直接返回，防止后续访问空定时器句柄。 */
    if ((cfg == 0) || (cfg->htim == 0)) {
        return;
    }

    /*
     * 把不同 APB 总线上的定时器统一分频到 MOTOR_COUNTER_FREQ_HZ。
     * 上层设置 step_hz 时，只需要用 1MHz / step_hz 计算 ARR。
     */
    psc = cfg->timer_clk_hz / MOTOR_COUNTER_FREQ_HZ;
    if (psc == 0U) {
        psc = 1U;
    }

    __HAL_TIM_SET_PRESCALER(cfg->htim, psc - 1U);
    cfg->htim->Instance->EGR = TIM_EGR_UG;
}

void MotorHw_Init(void)
{
    uint32_t i;

    /* 初始化时逐个电机复位 PWM 状态、设置默认频率、默认方向，并关闭输出。 */
    for (i = 0; i < MOTOR_COUNT; i++) {
        g_pwm_started[i] = 0U;
        motor_set_counter_to_1mhz(&g_motor_cfg[i]);
        (void)MotorHw_SetStepFreq((MotorId_t)i, 200U);
        MotorHw_SetDirection((MotorId_t)i, MOTOR_DIR_CW);
        MotorHw_SetEnable((MotorId_t)i, false);
        __HAL_TIM_DISABLE_IT(g_motor_cfg[i].htim, TIM_IT_UPDATE);
        __HAL_TIM_CLEAR_FLAG(g_motor_cfg[i].htim, TIM_FLAG_UPDATE);
    }
}

const MotorHwConfig_t *MotorHw_GetConfig(MotorId_t id)
{
    /* 返回只读硬件配置，供中断处理或调试代码查询定时器和 GPIO 映射。 */
    if (!motor_id_valid(id)) {
        return 0;
    }
    return &g_motor_cfg[id];
}

MotorDir_t MotorHw_GetPositiveDirection(MotorId_t id)
{
    /* 实机定义：包括 M4 在内，各轴 CW 均为坐标/编码器正方向。 */
    (void)id;
    return MOTOR_DIR_CW;
}

bool MotorHw_DirectionIncreasesPosition(MotorId_t id, MotorDir_t dir)
{
    if (!motor_id_valid(id)) {
        return false;
    }
    return (dir == MotorHw_GetPositiveDirection(id));
}

MotorDir_t MotorHw_GetDirectionForPositionChange(MotorId_t id, bool increase)
{
    MotorDir_t positive_dir = MotorHw_GetPositiveDirection(id);

    if (increase) {
        return positive_dir;
    }
    return (positive_dir == MOTOR_DIR_CW) ? MOTOR_DIR_CCW : MOTOR_DIR_CW;
}

MotorId_t MotorHw_FromTim(TIM_HandleTypeDef *htim)
{
    uint32_t i;

    /*
     * 定时器更新中断只会给出 TIM_HandleTypeDef。
     * 这里反查出对应的 MotorId_t，方便 MotorCtrl_OnTimUpdate() 统计步数。
     */
    for (i = 0; i < MOTOR_COUNT; i++) {
        if (g_motor_cfg[i].htim == htim) {
            return (MotorId_t)i;
        }
    }

    return MOTOR_ID_INVALID;
}

void MotorHw_SetDirection(MotorId_t id, MotorDir_t dir)
{
    GPIO_PinState state;

    /* 方向引脚只表达正转/反转，具体正方向由机械安装和接线决定。 */
    if (!motor_id_valid(id)) {
        return;
    }

    state = (dir == MOTOR_DIR_CW) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(g_motor_cfg[id].dir_port, g_motor_cfg[id].dir_pin, state);
}

void MotorHw_SetEnable(MotorId_t id, bool enable)
{
    GPIO_PinState state;
    bool active_low;

    if (!motor_id_valid(id)) {
        return;
    }

    /* 根据每个电机单独配置的使能极性，换算成实际要写入 GPIO 的电平。 */
    active_low = (g_enable_active_low[id] != 0U);
    if (enable) {
        state = active_low ? GPIO_PIN_RESET : GPIO_PIN_SET;
    } else {
        state = active_low ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    HAL_GPIO_WritePin(g_motor_cfg[id].en_port, g_motor_cfg[id].en_pin, state);
}

void MotorHw_StartPulse(MotorId_t id)
{
    TIM_HandleTypeDef *htim;
    TIM_TypeDef *tim;

    if (!motor_id_valid(id)) {
        return;
    }

    /* 已经启动过的 PWM 不重复启动，避免重复打开中断或扰动定时器状态。 */
    if (g_pwm_started[id] != 0U) {
        return;
    }

    htim = g_motor_cfg[id].htim;
    tim = htim->Instance;

    /* 启动前清掉历史更新标志，并打开更新中断用于上层计步。 */
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);

    /* 当前硬件映射只使用 CH1；如果配置异常，关闭刚打开的更新中断后退出。 */
    if (g_motor_cfg[id].channel != TIM_CHANNEL_1) {
        __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
        return;
    }

    /* 高级定时器需要打开 MOE 主输出，否则即使计数器运行也不会真正输出 PWM。 */
    tim->CCER |= TIM_CCER_CC1E;
    if (IS_TIM_BREAK_INSTANCE(tim) != RESET) {
        tim->BDTR |= TIM_BDTR_MOE;
    }
    __HAL_TIM_ENABLE(htim);
    g_pwm_started[id] = 1U;
}

void MotorHw_StopPulse(MotorId_t id)
{
    TIM_HandleTypeDef *htim;
    TIM_TypeDef *tim;

    if (!motor_id_valid(id)) {
        return;
    }

    /* 未启动的 PWM 不重复停止，保持接口幂等。 */
    if (g_pwm_started[id] == 0U) {
        return;
    }

    /* 停止 PWM 时同时关闭更新中断，避免上层继续收到无意义的计步事件。 */
    htim = g_motor_cfg[id].htim;
    tim = htim->Instance;
    __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);

    if (g_motor_cfg[id].channel == TIM_CHANNEL_1) {
        tim->CCER &= ~TIM_CCER_CC1E;
    }
    if (IS_TIM_BREAK_INSTANCE(tim) != RESET) {
        tim->BDTR &= ~TIM_BDTR_MOE;
    }
    __HAL_TIM_DISABLE(htim);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    g_pwm_started[id] = 0U;
}

bool MotorHw_SetStepFreq(MotorId_t id, uint32_t step_hz)
{
    uint32_t arr;
    const MotorHwConfig_t *cfg;

    if (!motor_id_valid(id)) {
        return false;
    }

    /* 频率必须在驱动器和定时器允许范围内，超出范围直接拒绝。 */
    if ((step_hz < MOTOR_STEP_FREQ_MIN_HZ) || (step_hz > MOTOR_STEP_FREQ_MAX_HZ)) {
        return false;
    }

    cfg = &g_motor_cfg[id];
    if (cfg->htim == 0) {
        return false;
    }

    /*
     * 统一 1MHz 计数频率后：
     * ARR + 1 = 1,000,000 / step_hz；
     * CCR 设置为周期的一半，得到约 50% 占空比的 STEP 脉冲。
     */
    arr = MOTOR_COUNTER_FREQ_HZ / step_hz;
    if (arr == 0U) {
        return false;
    }
    arr -= 1U;

    __HAL_TIM_SET_AUTORELOAD(cfg->htim, arr);
    __HAL_TIM_SET_COMPARE(cfg->htim, cfg->channel, (arr + 1U) / 2U);
    return true;
}
