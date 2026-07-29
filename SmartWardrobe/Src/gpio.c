/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   GPIO initialization
  ******************************************************************************
  */
/* USER CODE END Header */

#include "gpio.h"

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* 输出默认电平：先给 LED、继电器、方向、蜂鸣器和使能脚一个确定状态。 */
  HAL_GPIO_WritePin(GPIOD, LED1_Pin | LED2_Pin | LED3_Pin | ELM_CT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, E1_DIR_Pin | E1_ENABLE_Pin | BUZZ_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, E2_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(E3_DIR_GPIO_Port, E3_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(E3_ENABLE_GPIO_Port, E3_ENABLE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, E4_DIR_Pin | E4_ENABLE_Pin | E5_ENABLE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, E5_DIR_Pin, GPIO_PIN_RESET);

  /* M1~M4 使能为低电平有效，上电默认写高电平保持关闭。 */
  HAL_GPIO_WritePin(GPIOC, E1_ENABLE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, E2_ENABLE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(E3_ENABLE_GPIO_Port, E3_ENABLE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, E4_ENABLE_Pin, GPIO_PIN_SET);
  /* M5 夹爪电机同样为低电平使能，上电默认关闭。 */
  HAL_GPIO_WritePin(GPIOB, E5_ENABLE_Pin, GPIO_PIN_SET);

  /* LED 输出引脚。 */
  GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin | LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* 电磁铁继电器控制输出，软件默认释放；硬件仍应保留栅极下拉。 */
  GPIO_InitStruct.Pin = ELM_CT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ELM_CT_GPIO_Port, &GPIO_InitStruct);

  /* M1 方向/使能引脚，以及蜂鸣器输出。 */
  GPIO_InitStruct.Pin = E1_DIR_Pin | E1_ENABLE_Pin | BUZZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* M2 方向/使能引脚。 */
  GPIO_InitStruct.Pin = E2_DIR_Pin | E2_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* M3 方向/使能引脚。 */
  GPIO_InitStruct.Pin = E3_DIR_Pin | E3_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(E3_DIR_GPIO_Port, &GPIO_InitStruct);

  /* M4 方向/使能，以及 M5 使能引脚。 */
  GPIO_InitStruct.Pin = E4_DIR_Pin | E4_ENABLE_Pin | E5_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* M5 方向引脚。 */
  GPIO_InitStruct.Pin = E5_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EX_KEY1~EX_KEY4：NPN 光电传感器输入，被遮挡时为低电平。 */
  GPIO_InitStruct.Pin = EX_KEY1_Pin | EX_KEY2_Pin | EX_KEY3_Pin | EX_KEY4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EX_KEY1_GPIO_Port, &GPIO_InitStruct);

  /* 板载按键输入。 */
  GPIO_InitStruct.Pin = KEY1_Pin | KEY2_Pin | KEY3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}
