/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   TIM peripheral configuration
  ******************************************************************************
  */
/* USER CODE END Header */

#include "tim.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;

/*
 * 编码器定时器公共初始化：
 * TIM1/TIM2/TIM3/TIM4/TIM5 使用 TI1+TI2 正交编码器模式读取增量编码器。
 */
static void Tim_EncoderCommonInit(TIM_HandleTypeDef *htim, TIM_TypeDef *instance, uint32_t period)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim->Instance = instance;
  htim->Init.Prescaler = 0;
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = period;
  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim->Init.RepetitionCounter = 0;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 6;

  if (HAL_TIM_Encoder_Init(htim, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Tim_PwmCommonInit(TIM_HandleTypeDef *htim, TIM_TypeDef *instance, uint32_t prescaler)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /*
   * 步进电机 STEP PWM 公共初始化：
   * TIM8/TIM9/TIM10/TIM11/TIM12 均使用 CH1 输出 STEP 脉冲。
   * motor_hw.c 后续会重新设置预分频、ARR 和 CCR 来得到目标 step_hz。
   */
  htim->Instance = instance;
  htim->Init.Prescaler = prescaler;
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = 1000 - 1;
  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim->Init.RepetitionCounter = 0;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(htim) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(htim) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (instance == TIM8)
  {
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

    if (HAL_TIMEx_ConfigBreakDeadTime(htim, &sBreakDeadTimeConfig) != HAL_OK)
    {
      Error_Handler();
    }
  }

  HAL_TIM_MspPostInit(htim);
}

void MX_TIM1_Init(void)
{
  /* TIM1 用作 M1 增量编码器。 */
  Tim_EncoderCommonInit(&htim1, TIM1, 0xFFFF);
}

void MX_TIM2_Init(void)
{
  /* TIM2 用作 M2 增量编码器/预留编码器输入，32 位计数。 */
  Tim_EncoderCommonInit(&htim2, TIM2, 0xFFFFFFFFUL);
}

void MX_TIM3_Init(void)
{
  /* TIM3 用作 M3 增量编码器。 */
  Tim_EncoderCommonInit(&htim3, TIM3, 0xFFFF);
}

void MX_TIM4_Init(void)
{
  /* TIM4 用作 M4 增量编码器。 */
  Tim_EncoderCommonInit(&htim4, TIM4, 0xFFFF);
}

void MX_TIM5_Init(void)
{
  /* TIM5 当前作为编码器输入预留，32 位计数。 */
  Tim_EncoderCommonInit(&htim5, TIM5, 0xFFFFFFFFUL);
}

void MX_TIM8_Init(void)
{
  /* TIM8 CH1 输出 M1 STEP。 */
  Tim_PwmCommonInit(&htim8, TIM8, 168 - 1);
}

void MX_TIM9_Init(void)
{
  /* TIM9 CH1 输出 M2 STEP。 */
  Tim_PwmCommonInit(&htim9, TIM9, 168 - 1);
}

void MX_TIM10_Init(void)
{
  /* TIM10 CH1 输出 M3 STEP。 */
  Tim_PwmCommonInit(&htim10, TIM10, 168 - 1);
}

void MX_TIM11_Init(void)
{
  /* TIM11 CH1 输出 M4 STEP。 */
  Tim_PwmCommonInit(&htim11, TIM11, 168 - 1);
}

void MX_TIM12_Init(void)
{
  /* TIM12 CH1 输出 M5 STEP，TIM12 位于 APB1，所以预分频基于 84MHz。 */
  Tim_PwmCommonInit(&htim12, TIM12, 84 - 1);
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *tim_encoderHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 根据具体 TIM 实例配置对应的编码器 A/B 相 GPIO。 */
  if (tim_encoderHandle->Instance == TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
  else if (tim_encoderHandle->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else if (tim_encoderHandle->Instance == TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if (tim_encoderHandle->Instance == TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  }
  else if (tim_encoderHandle->Instance == TIM5)
  {
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle)
{
  /* 为 STEP PWM 定时器打开时钟，并打开更新中断供 MotorCtrl 计步。 */
  if (tim_baseHandle->Instance == TIM8)
  {
    __HAL_RCC_TIM8_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
    HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
  }
  else if (tim_baseHandle->Instance == TIM9)
  {
    __HAL_RCC_TIM9_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  }
  else if (tim_baseHandle->Instance == TIM10)
  {
    __HAL_RCC_TIM10_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  }
  else if (tim_baseHandle->Instance == TIM11)
  {
    __HAL_RCC_TIM11_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  }
  else if (tim_baseHandle->Instance == TIM12)
  {
    __HAL_RCC_TIM12_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 配置各个 STEP 输出引脚的定时器复用功能。 */
  if (timHandle->Instance == TIM8)
  {
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = E1_ST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(E1_ST_GPIO_Port, &GPIO_InitStruct);
  }
  else if (timHandle->Instance == TIM9)
  {
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = E2_ST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
    HAL_GPIO_Init(E2_ST_GPIO_Port, &GPIO_InitStruct);
  }
  else if (timHandle->Instance == TIM10)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = E3_ST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;
    HAL_GPIO_Init(E3_ST_GPIO_Port, &GPIO_InitStruct);
  }
  else if (timHandle->Instance == TIM11)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = E4_ST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
    HAL_GPIO_Init(E4_ST_GPIO_Port, &GPIO_InitStruct);
  }
  else if (timHandle->Instance == TIM12)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = E5_ST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
    HAL_GPIO_Init(E5_ST_GPIO_Port, &GPIO_InitStruct);
  }
}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *tim_encoderHandle)
{
  if (tim_encoderHandle->Instance == TIM1)
  {
    __HAL_RCC_TIM1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_9 | GPIO_PIN_11);
  }
  else if (tim_encoderHandle->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);
  }
  else if (tim_encoderHandle->Instance == TIM3)
  {
    __HAL_RCC_TIM3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6 | GPIO_PIN_7);
  }
  else if (tim_encoderHandle->Instance == TIM4)
  {
    __HAL_RCC_TIM4_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12 | GPIO_PIN_13);
  }
  else if (tim_encoderHandle->Instance == TIM5)
  {
    __HAL_RCC_TIM5_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *tim_baseHandle)
{
  if (tim_baseHandle->Instance == TIM8)
  {
    __HAL_RCC_TIM8_CLK_DISABLE();
  }
  else if (tim_baseHandle->Instance == TIM9)
  {
    __HAL_RCC_TIM9_CLK_DISABLE();
  }
  else if (tim_baseHandle->Instance == TIM10)
  {
    __HAL_RCC_TIM10_CLK_DISABLE();
  }
  else if (tim_baseHandle->Instance == TIM11)
  {
    __HAL_RCC_TIM11_CLK_DISABLE();
  }
  else if (tim_baseHandle->Instance == TIM12)
  {
    __HAL_RCC_TIM12_CLK_DISABLE();
  }
}
