/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_bldcm_control.h"
#include "arm_math.h"
#include <stdio.h>
#include <math.h>
#include "bsp_adc.h"
#include "delay.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "foc_algorithm.h"
#include "low_task.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOE
#define KEY3_Pin GPIO_PIN_13
#define KEY3_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOA
#define HALL_INPUTU_Pin GPIO_PIN_10
#define HALL_INPUTU_GPIO_Port GPIOH
#define HALL_INPUTV_Pin GPIO_PIN_11
#define HALL_INPUTV_GPIO_Port GPIOH
#define HALL_INPUTW_Pin GPIO_PIN_12
#define HALL_INPUTW_GPIO_Port GPIOH
#define KEY2_Pin GPIO_PIN_2
#define KEY2_GPIO_Port GPIOG
#define KEY4_Pin GPIO_PIN_3
#define KEY4_GPIO_Port GPIOG
#define KEY5_Pin GPIO_PIN_4
#define KEY5_GPIO_Port GPIOG
#define MOTOR_OCNPWM1_Pin GPIO_PIN_13
#define MOTOR_OCNPWM1_GPIO_Port GPIOH
#define MOTOR_OCNPWM2_Pin GPIO_PIN_14
#define MOTOR_OCNPWM2_GPIO_Port GPIOH
#define MOTOR_OCNPWM3_Pin GPIO_PIN_15
#define MOTOR_OCNPWM3_GPIO_Port GPIOH
#define LED0_Pin GPIO_PIN_15
#define LED0_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOG
#define DEBUG_USART_TX_Pin GPIO_PIN_6
#define DEBUG_USART_TX_GPIO_Port GPIOB
#define DEBUG_USART_RX_Pin GPIO_PIN_7
#define DEBUG_USART_RX_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOB
#define MOTOR_OCPWM1_Pin GPIO_PIN_5
#define MOTOR_OCPWM1_GPIO_Port GPIOI
#define MOTOR_OCPWM2_Pin GPIO_PIN_6
#define MOTOR_OCPWM2_GPIO_Port GPIOI
#define MOTOR_OCPWM3_Pin GPIO_PIN_7
#define MOTOR_OCPWM3_GPIO_Port GPIOI

/* USER CODE BEGIN Private defines */
#define LED_ON  GPIO_PIN_RESET
#define LED_OFF GPIO_PIN_SET

#define LED1(a)    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,a)

#define LED2(a)    HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,a)

#define LED3(a)    HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,a)

#define LED4(a)    HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,a)

#define    digitalHi(p, i)            {p->BSRR = i;}
#define digitalLo(p, i)            {p->BSRR=(uint32_t)i << 16;}
#define digitalToggle(p, i)        {p->ODR ^=i;}

#define LED0_TOGGLE        digitalToggle(LED0_GPIO_Port,LED0_Pin)
#define LED0_OFF        digitalHi(LED0_GPIO_Port,LED0_Pin)
#define LED0_ON            digitalLo(LED0_GPIO_Port,LED0_Pin)

#define LED1_TOGGLE        digitalToggle(LED1_GPIO_Port,LED1_Pin)
#define LED1_OFF        digitalHi(LED1_GPIO_Port,LED1_Pin)
#define LED1_ON            digitalLo(LED1_GPIO_Port,LED1_Pin)

#define LED2_TOGGLE        digitalToggle(LED2_GPIO_Port,LED2_Pin)
#define LED2_OFF        digitalHi(LED2_GPIO_Port,LED2_Pin)
#define LED2_ON            digitalLo(LED2_GPIO_Port,LED2_Pin)

#define LED3_TOGGLE        digitalToggle(LED3_GPIO_Port,LED3_Pin)
#define LED3_OFF        digitalHi(LED3_GPIO_Port,LED3_Pin)
#define LED3_ON            digitalLo(LED3_GPIO_Port,LED3_Pin)


#define LED_1  \
                    LED1_ON;\
                    LED2_OFF\
                    LED3_OFF\
          LED4_OFF

#define LED_2        \
                    LED1_OFF;\
                    LED2_ON\
                    LED3_OFF\
          LED4_OFF

#define LED_3    \
                    LED1_OFF;\
                    LED2_OFF\
                    LED3_ON\
          LED4_OFF

#define LED_4    \
                    LED1_OFF;\
                    LED2_OFF\
          LED3_OFF\
                    LED4_ON


#define LED_RGBOFF\
                    LED1_OFF;\
                    LED2_OFF\
                    LED3_OFF\
          LED4_OFF
#define KEY_ON    1
#define KEY_OFF    0
#define PWM_PERIOD_COUNT     (5600)

#define PWM_MAX_PERIOD_COUNT    (PWM_PERIOD_COUNT - 100)

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

#define HALL_TIM_CLOCK (u32)90000000
#define HALL_SAMPLE_FREQ (u32)10000
#define PHASE_SHIFT_ANGLE (float)(60.0f/360.0f*2.0f*PI)         //单位角度
#define HALL_ANGLE_FACTOR (float)((float)HALL_TIM_CLOCK/(float)HALL_SAMPLE_FREQ*PI/3.0f)
#define HALL_SPEED_FACTOR (float)((float)HALL_TIM_CLOCK/6.0f)

#define FOC_PERIOD          0.0001F
#define MOTOR_STARTUP_CURRENT   1.0f
#define SPEED_LOOP_CLOSE_RAD_S  50.0f  // 闭环速度控制速度值 rad/s


#define HALL_FOC_SELECT
//#define SENSORLESS_FOC_SELECT


#define RS_PARAMETER     0.59f            //电阻参数
#define LS_PARAMETER     0.001f           //电感参数

#define FLUX_PARAMETER   0.01150f         //磁链参数


#define PWM_TIM_CLOCK       168000000

#define PWM_TIM_FREQ        10000         //HZ
#define PWM_TIM_PULSE       (PWM_TIM_CLOCK/(2*PWM_TIM_FREQ))
#define PWM_TIM_PULSE_TPWM  (PWM_TIM_CLOCK/(PWM_TIM_FREQ))
#define DEAD_TIME         ((u16) 5)
#define PWM_DEAD_TIME     (u16)((unsigned long long)PWM_TIM_CLOCK/2*(unsigned long long)DEAD_TIME/1000000000uL)

#define KEY1_INT_IRQn                   EXTI0_IRQn
#define KEY1_INT_IRQHandler             EXTI0_IRQHandler
#define KEY2_INT_IRQn                   EXTI2_IRQn
#define KEY2_INT_IRQHandler             EXTI2_IRQHandler
#define KEY3_INT_IRQn                   EXTI15_10_IRQn
#define KEY3_INT_IRQHandler             EXTI15_10_IRQHandler

#define KEY1 HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) /* 读取 KEY0 引脚 */
#define KEY2 HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) /* 读取 KEY1 引脚 */
#define KEY3 HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) /* 读取 WKUP 引脚 */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
