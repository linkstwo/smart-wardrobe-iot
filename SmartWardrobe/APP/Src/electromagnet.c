#include "electromagnet.h"

#include "main.h"

static uint8_t g_electromagnet_on;

void Electromagnet_Init(void)
{
    /* GPIO 初始化已经预置低电平，这里再次关闭作为上电软件兜底。 */
    HAL_GPIO_WritePin(ELM_CT_GPIO_Port, ELM_CT_Pin, GPIO_PIN_RESET);
    g_electromagnet_on = 0U;
}

int Electromagnet_Set(uint8_t on)
{
    GPIO_PinState state = (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    HAL_GPIO_WritePin(ELM_CT_GPIO_Port, ELM_CT_Pin, state);
    g_electromagnet_on = (on != 0U) ? 1U : 0U;
    return 0;
}

void Electromagnet_ForceOff(void)
{
    /* 急停/HardFault 路径调用时不依赖 HAL 状态。 */
    ELM_CT_GPIO_Port->BSRR = ((uint32_t)ELM_CT_Pin << 16U);
    g_electromagnet_on = 0U;
}

uint8_t Electromagnet_IsOn(void)
{
    return g_electromagnet_on;
}

uint8_t Electromagnet_PinHigh(void)
{
    return (HAL_GPIO_ReadPin(ELM_CT_GPIO_Port, ELM_CT_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}
