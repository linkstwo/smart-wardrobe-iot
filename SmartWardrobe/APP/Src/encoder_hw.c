#include "encoder_hw.h"

#include "tim.h"

/*
 * 增量编码器硬件层：
 * 负责把电机编号映射到具体编码器定时器，并提供计数器读取、方向读取、AB 相电平读取。
 * 注意：这里读的是底层硬件计数值；累计位移、溢出处理和状态快照在 encoder_ctrl.c 中完成。
 */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint8_t bits;
    GPIO_TypeDef *a_port;
    uint16_t a_pin;
    GPIO_TypeDef *b_port;
    uint16_t b_pin;
} EncoderHwMap_t;

/*
 * 编码器映射表：
 * M1 -> TIM1，16 位计数；
 * M2 -> TIM2，32 位计数；
 * M3 -> TIM3，16 位计数；
 * M4 -> TIM4，16 位计数；
 * M5 当前没有增量编码器，所以句柄和 GPIO 都为 0。
 */
static const EncoderHwMap_t g_encoder_map[MOTOR_COUNT] = {
    {&htim1, 16U, GPIOE, GPIO_PIN_9, GPIOE, GPIO_PIN_11},
    {&htim2, 32U, GPIOA, GPIO_PIN_5, GPIOB, GPIO_PIN_3},
    {&htim3, 16U, GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_7},
    {&htim4, 16U, GPIOD, GPIO_PIN_12, GPIOD, GPIO_PIN_13},
    {0, 0U, 0, 0U, 0, 0U}
};

static bool encoder_id_valid(MotorId_t id)
{
    /* 编码器数组仍按 MOTOR_COUNT 建表，因此先检查电机编号范围。 */
    return (id < MOTOR_COUNT);
}

void EncoderHw_Init(void)
{
    uint32_t i;

    /* 启动所有已配置的编码器接口，并把硬件计数器清零。 */
    for (i = 0U; i < MOTOR_COUNT; i++) {
        if (g_encoder_map[i].htim == 0) {
            continue;
        }
        (void)HAL_TIM_Encoder_Start(g_encoder_map[i].htim, TIM_CHANNEL_ALL);
        __HAL_TIM_SET_COUNTER(g_encoder_map[i].htim, 0U);
    }
}

uint32_t EncoderHw_ReadCounter(MotorId_t id)
{
    /* 未配置编码器的电机返回 0，调用方可结合 bits 或状态判断是否有效。 */
    if (!encoder_id_valid(id)) {
        return 0U;
    }

    if (g_encoder_map[id].htim == 0) {
        return 0U;
    }

    return __HAL_TIM_GET_COUNTER(g_encoder_map[id].htim);
}

bool EncoderHw_IsCountingDown(MotorId_t id)
{
    /* 读取定时器方向位，用于判断当前编码器计数正在增加还是减少。 */
    if (!encoder_id_valid(id)) {
        return false;
    }

    if (g_encoder_map[id].htim == 0) {
        return false;
    }

    return (__HAL_TIM_IS_TIM_COUNTING_DOWN(g_encoder_map[id].htim) != RESET);
}

uint8_t EncoderHw_GetCounterBits(MotorId_t id)
{
    /* 返回硬件计数器位宽，控制层用它处理 16 位/32 位计数器回绕。 */
    if (!encoder_id_valid(id)) {
        return 0U;
    }

    return g_encoder_map[id].bits;
}

int EncoderHw_Clear(MotorId_t id)
{
    /* 清零单个编码器硬件计数器，常用于回零后建立新的当前位置基准。 */
    if (!encoder_id_valid(id)) {
        return -1;
    }

    if (g_encoder_map[id].htim == 0) {
        return -1;
    }

    __HAL_TIM_SET_COUNTER(g_encoder_map[id].htim, 0U);
    return 0;
}

int EncoderHw_ReadAB(MotorId_t id, bool *a_high, bool *b_high)
{
    /* 直接读取编码器 A/B 相 GPIO 电平，主要用于调试接线和判断输入状态。 */
    if ((!encoder_id_valid(id)) || (a_high == 0) || (b_high == 0) ||
        (g_encoder_map[id].a_port == 0) || (g_encoder_map[id].b_port == 0)) {
        return -1;
    }

    *a_high = (HAL_GPIO_ReadPin(g_encoder_map[id].a_port, g_encoder_map[id].a_pin) == GPIO_PIN_SET);
    *b_high = (HAL_GPIO_ReadPin(g_encoder_map[id].b_port, g_encoder_map[id].b_pin) == GPIO_PIN_SET);
    return 0;
}
