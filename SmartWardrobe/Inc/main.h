/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Error_Handler(void);

/* 电机 STEP 脉冲输出引脚，分别对应 M1~M5。 */
#define E1_ST_Pin GPIO_PIN_6
#define E1_ST_GPIO_Port GPIOC
#define E2_ST_Pin GPIO_PIN_5
#define E2_ST_GPIO_Port GPIOE
#define E3_ST_Pin GPIO_PIN_8
#define E3_ST_GPIO_Port GPIOB
#define E4_ST_Pin GPIO_PIN_9
#define E4_ST_GPIO_Port GPIOB
#define E5_ST_Pin GPIO_PIN_14
#define E5_ST_GPIO_Port GPIOB

/* 电机方向 DIR 引脚，分别对应 M1~M5。 */
#define E1_DIR_Pin GPIO_PIN_8
#define E1_DIR_GPIO_Port GPIOC
#define E2_DIR_Pin GPIO_PIN_7
#define E2_DIR_GPIO_Port GPIOE
#define E3_DIR_Pin GPIO_PIN_12
#define E3_DIR_GPIO_Port GPIOA
#define E4_DIR_Pin GPIO_PIN_7
#define E4_DIR_GPIO_Port GPIOB
#define E5_DIR_Pin GPIO_PIN_0
#define E5_DIR_GPIO_Port GPIOD

/* 电机使能 EN 引脚：
 * M1~M5 均为低电平使能；
 * M5 虽然用于夹爪，但仍走 step/dir/enable 电机控制链路。
 */
#define E1_ENABLE_Pin GPIO_PIN_7
#define E1_ENABLE_GPIO_Port GPIOC
#define E2_ENABLE_Pin GPIO_PIN_6
#define E2_ENABLE_GPIO_Port GPIOE
#define E3_ENABLE_Pin GPIO_PIN_11
#define E3_ENABLE_GPIO_Port GPIOA
#define E4_ENABLE_Pin GPIO_PIN_6
#define E4_ENABLE_GPIO_Port GPIOB
#define E5_ENABLE_Pin GPIO_PIN_15
#define E5_ENABLE_GPIO_Port GPIOB

/* EX_KEY1~EX_KEY4：光电/限位传感器输入，被遮挡时为低电平。 */
#define EX_KEY1_Pin GPIO_PIN_0
#define EX_KEY1_GPIO_Port GPIOC
#define EX_KEY2_Pin GPIO_PIN_1
#define EX_KEY2_GPIO_Port GPIOC
#define EX_KEY3_Pin GPIO_PIN_2
#define EX_KEY3_GPIO_Port GPIOC
#define EX_KEY4_Pin GPIO_PIN_3
#define EX_KEY4_GPIO_Port GPIOC

/* 板载 LED 输出。 */
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOD

/* 电磁铁继电器控制：PD8 高电平吸合，低电平释放。 */
#define ELM_CT_Pin GPIO_PIN_8
#define ELM_CT_GPIO_Port GPIOD

/* 兼容旧工程的 LED/按键别名，旧板定义过 5 个 LED 和 4 个按键。 */
#define LED4_Pin LED1_Pin
#define LED4_GPIO_Port LED1_GPIO_Port
#define LED5_Pin LED2_Pin
#define LED5_GPIO_Port LED2_GPIO_Port

/* 板载按键输入。 */
#define KEY1_Pin GPIO_PIN_4
#define KEY1_GPIO_Port GPIOD
#define KEY2_Pin GPIO_PIN_5
#define KEY2_GPIO_Port GPIOD
#define KEY3_Pin GPIO_PIN_6
#define KEY3_GPIO_Port GPIOD
#define KEY4_Pin KEY3_Pin
#define KEY4_GPIO_Port KEY3_GPIO_Port

/* 蜂鸣器输出。 */
#define BUZZ_Pin GPIO_PIN_11
#define BUZZ_GPIO_Port GPIOC

/* 兼容旧工程拼写的别名：PLUSE/BEZZ 保持可用，实际仍映射到当前 STEP/BUZZ。 */
#define E1_PLUSE_Pin E1_ST_Pin
#define E1_PLUSE_GPIO_Port E1_ST_GPIO_Port
#define E2_PLUSE_Pin E2_ST_Pin
#define E2_PLUSE_GPIO_Port E2_ST_GPIO_Port
#define E3_PLUSE_Pin E3_ST_Pin
#define E3_PLUSE_GPIO_Port E3_ST_GPIO_Port
#define E4_PLUSE_Pin E4_ST_Pin
#define E4_PLUSE_GPIO_Port E4_ST_GPIO_Port
#define E5_PLUSE_Pin E5_ST_Pin
#define E5_PLUSE_GPIO_Port E5_ST_GPIO_Port
#define BEZZ_Pin BUZZ_Pin
#define BEZZ_GPIO_Port BUZZ_GPIO_Port

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
