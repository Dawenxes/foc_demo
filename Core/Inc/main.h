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


#define LED1_TOGGLE        digitalToggle(LED1_GPIO_Port,LED1_Pin)
#define LED1_OFF        digitalHi(LED1_GPIO_Port,LED1_Pin)
#define LED1_ON            digitalLo(LED1_GPIO_Port,LED1_Pin)

#define LED2_TOGGLE        digitalToggle(LED2_GPIO_Port,LED2_Pin)
#define LED2_OFF        digitalHi(LED2_GPIO_Port,LED2_Pin)
#define LED2_ON            digitalLo(LED2_GPIO_Port,LED2_Pin)

#define LED3_TOGGLE        digitalToggle(LED3_GPIO_Port,LED3_Pin)
#define LED3_OFF        digitalHi(LED3_GPIO_Port,LED3_Pin)
#define LED3_ON            digitalLo(LED3_GPIO_Port,LED3_Pin)

#define LED4_TOGGLE        digitalToggle(LED4_GPIO_Port,LED4_Pin)
#define LED4_OFF        digitalHi(LED4_GPIO_Port,LED4_Pin)
#define LED4_ON            digitalLo(LED4_GPIO_Port,LED4_Pin)

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
#define PHASE_SHIFT_ANGLE (float)(60.0f/360.0f*2.0f*PI)         //µ¥Î»½Ç¶È
#define HALL_ANGLE_FACTOR (float)((float)HALL_TIM_CLOCK/(float)HALL_SAMPLE_FREQ*PI/3.0f)
#define HALL_SPEED_FACTOR (float)((float)HALL_TIM_CLOCK/6.0f)

#define FOC_PERIOD          0.0001F
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
