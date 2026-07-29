/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "stm32f4xx_it.h"
#ifndef ENCODER_TEST_ONLY
#include "motor_ctrl.h"
#endif

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;

extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;

static uint8_t tim_update_event_pending(TIM_HandleTypeDef *htim)
{
#ifdef ENCODER_TEST_ONLY
  (void)htim;
  return 0U;
#else
  /*
   * 在调用 HAL_TIM_IRQHandler() 之前判断更新中断是否挂起。
   * HAL 处理后标志会被清掉，所以必须先记录，后面才能决定是否调用 MotorCtrl_OnTimUpdate() 计步。
   */
  if ((htim == 0) || (htim->Instance == 0)) {
    return 0U;
  }

  if ((__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) &&
      (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET)) {
    return 1U;
  }

  return 0U;
#endif
}

void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  /* Fault-safe motor shutdown: disable STEP timers before entering the trap. */
  __disable_irq();

  TIM8->DIER &= ~TIM_DIER_UIE;
  TIM8->CCER &= ~TIM_CCER_CC1E;
  TIM8->BDTR &= ~TIM_BDTR_MOE;
  TIM8->CR1 &= ~TIM_CR1_CEN;

  TIM9->DIER &= ~TIM_DIER_UIE;
  TIM9->CCER &= ~TIM_CCER_CC1E;
  TIM9->CR1 &= ~TIM_CR1_CEN;
  TIM10->DIER &= ~TIM_DIER_UIE;
  TIM10->CCER &= ~TIM_CCER_CC1E;
  TIM10->CR1 &= ~TIM_CR1_CEN;
  TIM11->DIER &= ~TIM_DIER_UIE;
  TIM11->CCER &= ~TIM_CCER_CC1E;
  TIM11->CR1 &= ~TIM_CR1_CEN;
  TIM12->DIER &= ~TIM_DIER_UIE;
  TIM12->CCER &= ~TIM_CCER_CC1E;
  TIM12->CR1 &= ~TIM_CR1_CEN;

  /* M1~M5 drivers are active-low, so setting EN high disables them. */
  E1_ENABLE_GPIO_Port->BSRR = E1_ENABLE_Pin;
  E2_ENABLE_GPIO_Port->BSRR = E2_ENABLE_Pin;
  E3_ENABLE_GPIO_Port->BSRR = E3_ENABLE_Pin;
  E4_ENABLE_GPIO_Port->BSRR = E4_ENABLE_Pin;
  E5_ENABLE_GPIO_Port->BSRR = E5_ENABLE_Pin;

  /* 电磁铁继电器高电平有效，HardFault 时必须强制释放。 */
  ELM_CT_GPIO_Port->BSRR = ((uint32_t)ELM_CT_Pin << 16U);

  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  /* HAL 1ms tick，主循环用 HAL_GetTick() 做软件调度。 */
  HAL_IncTick();
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* TIM9 更新中断用于 M2 STEP 计步；TIM1 同组中断保留给编码器/HAL 处理。 */
  uint8_t tim9_update = tim_update_event_pending(&htim9);

  HAL_TIM_IRQHandler(&htim9);
  HAL_TIM_IRQHandler(&htim1);

#ifndef ENCODER_TEST_ONLY
  if (tim9_update != 0U) {
    MotorCtrl_OnTimUpdate(&htim9);
  }
#else
  (void)tim9_update;
#endif
}

void TIM1_UP_TIM10_IRQHandler(void)
{
  /* TIM10 更新中断用于 M3 STEP 计步。 */
  uint8_t tim10_update = tim_update_event_pending(&htim10);

  HAL_TIM_IRQHandler(&htim10);
  HAL_TIM_IRQHandler(&htim1);

#ifndef ENCODER_TEST_ONLY
  if (tim10_update != 0U) {
    MotorCtrl_OnTimUpdate(&htim10);
  }
#else
  (void)tim10_update;
#endif
}

void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* TIM11 更新中断用于 M4 STEP 计步。 */
  uint8_t tim11_update = tim_update_event_pending(&htim11);

  HAL_TIM_IRQHandler(&htim11);
  HAL_TIM_IRQHandler(&htim1);

#ifndef ENCODER_TEST_ONLY
  if (tim11_update != 0U) {
    MotorCtrl_OnTimUpdate(&htim11);
  }
#else
  (void)tim11_update;
#endif
}

void TIM1_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}

void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim4);
}

void TIM5_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim5);
}

void UART4_IRQHandler(void)
{
  /* UART4 中断用于夹爪 Modbus 通信。 */
  HAL_UART_IRQHandler(&huart4);
}

void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* TIM12 更新中断用于 M5 STEP 计步；TIM8 的 break 事件同组交给 HAL 处理。 */
  uint8_t tim12_update = tim_update_event_pending(&htim12);

  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim12);

#ifndef ENCODER_TEST_ONLY
  if (tim12_update != 0U) {
    MotorCtrl_OnTimUpdate(&htim12);
  }
#else
  (void)tim12_update;
#endif
}

void TIM8_UP_TIM13_IRQHandler(void)
{
  /* TIM8 更新中断用于 M1 STEP 计步。 */
  uint8_t tim8_update = tim_update_event_pending(&htim8);

  HAL_TIM_IRQHandler(&htim8);

#ifndef ENCODER_TEST_ONLY
  if (tim8_update != 0U) {
    MotorCtrl_OnTimUpdate(&htim8);
  }
#else
  (void)tim8_update;
#endif
}

void USART1_IRQHandler(void)
{
  /* USART1 handles JH-AI-1811 Modbus requests and responses. */
  HAL_UART_IRQHandler(&huart1);
}

void USART2_IRQHandler(void)
{
  /* USART2 中断用于 RS485 绝对值编码器通信。 */
  HAL_UART_IRQHandler(&huart2);
}

void USART3_IRQHandler(void)
{
  /* USART3 中断用于 ESP32 通信和文本调试命令。 */
  HAL_UART_IRQHandler(&huart3);
}

void DMA1_Stream5_IRQHandler(void)
{
  /* USART2_RX DMA 中断。 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

void DMA1_Stream6_IRQHandler(void)
{
  /* USART2_TX DMA 中断。 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}
