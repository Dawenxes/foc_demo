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
#include "queue.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HALL1_C_Pin GPIO_PIN_15
#define HALL1_C_GPIO_Port GPIOC
#define HALL1_C_EXTI_IRQn EXTI15_10_IRQn
#define ENCODE_A_Pin GPIO_PIN_2
#define ENCODE_A_GPIO_Port GPIOB
#define ENCODE_A_EXTI_IRQn EXTI2_IRQn
#define HALL0_C_Pin GPIO_PIN_9
#define HALL0_C_GPIO_Port GPIOC
#define HALL0_C_EXTI_IRQn EXTI9_5_IRQn
#define RUN_LED_Pin GPIO_PIN_2
#define RUN_LED_GPIO_Port GPIOD
#define ENCODE_B_Pin GPIO_PIN_3
#define ENCODE_B_GPIO_Port GPIOB
#define ENCODE_B_EXTI_IRQn EXTI3_IRQn
#define HALL0_A_Pin GPIO_PIN_4
#define HALL0_A_GPIO_Port GPIOB
#define HALL0_A_EXTI_IRQn EXTI4_IRQn
#define HALL0_B_Pin GPIO_PIN_5
#define HALL0_B_GPIO_Port GPIOB
#define HALL0_B_EXTI_IRQn EXTI9_5_IRQn
#define HALL1_A_Pin GPIO_PIN_6
#define HALL1_A_GPIO_Port GPIOB
#define HALL1_A_EXTI_IRQn EXTI9_5_IRQn
#define HALL1_B_Pin GPIO_PIN_7
#define HALL1_B_GPIO_Port GPIOB
#define HALL1_B_EXTI_IRQn EXTI9_5_IRQn

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

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

#define HALL_TIM_CLOCK (u32)84000000
#define HALL_SAMPLE_FREQ (u32)10000
#define PHASE_SHIFT_ANGLE (float)(60.0f/360.0f*2.0f*PI)         //ƫ�ƽǶ�
#define HALL_ANGLE_FACTOR (float)((float)HALL_TIM_CLOCK/(float)HALL_SAMPLE_FREQ*PI/3.0f)
#define HALL_SPEED_FACTOR (float)((float)HALL_TIM_CLOCK/6.0f)

#define FOC_PERIOD          0.0001F
#define MOTOR_STARTUP_CURRENT   1.0f
#define SPEED_LOOP_CLOSE_RAD_S  50.0f  // �ջ��ٶȿ����ٶ�ֵ rad/s


#define HALL_FOC_SELECT
//#define SENSORLESS_FOC_SELECT


#define RS_PARAMETER     0.18f            //�������
#define LS_PARAMETER     0.0025f           //��в���
#define FLUX_PARAMETER   0.0160f         //��������


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

#define KEY1 HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) /* ��ȡ KEY0 ���� */
#define KEY2 HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) /* ��ȡ KEY1 ���� */
#define KEY3 HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) /* ��ȡ WKUP ���� */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
