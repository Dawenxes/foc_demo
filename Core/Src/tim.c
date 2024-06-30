/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
static uint16_t bldcm_pulse = 0;
/* USER CODE END 0 */

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
/* TIM5 init function */
void MX_TIM5_Init(void) {

    /* USER CODE BEGIN TIM5_Init 0 */

    /* USER CODE END TIM5_Init 0 */

    TIM_HallSensor_InitTypeDef sConfig = {0};
    TIM_IC_InitTypeDef icConfig = {0};
    TIM_SlaveConfigTypeDef slaveConfig = {0};

    /* USER CODE BEGIN TIM5_Init 1 */
    /* USER CODE END TIM5_Init 1 */
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 0xffffffff;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_IC_Init(&htim5) != HAL_OK) {
        Error_Handler();
    }
    icConfig.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    icConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
    icConfig.ICFilter = 0xF;
    icConfig.ICPrescaler = TIM_ICPSC_DIV1;
    if (HAL_TIM_IC_ConfigChannel(&htim5, &icConfig, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }


    /* USER CODE BEGIN TIM5_Init 2 */

    HAL_TIM_ConfigTI1Input(&htim5, TIM_TI1SELECTION_XORCOMBINATION);

    slaveConfig.InputTrigger= TIM_TS_TI1FP1;
    slaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    slaveConfig.TriggerPolarity= TIM_TRIGGERPOLARITY_BOTHEDGE;
    HAL_TIM_SlaveConfigSynchro(&htim5, &slaveConfig);
    //__HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);

    /* USER CODE END TIM5_Init 2 */

}

/* TIM8 init function */
void MX_TIM8_Init(void) {

    /* USER CODE BEGIN TIM8_Init 0 */

    /* USER CODE END TIM8_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM8_Init 1 */

    /* USER CODE END TIM8_Init 1 */
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim8.Init.Period = PWM_TIM_PULSE;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = PWM_TIM_PULSE >> 1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
    sBreakDeadTimeConfig.DeadTime = PWM_DEAD_TIME;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM8_Init 2 */
    /* USER CODE END TIM8_Init 2 */
    HAL_TIM_MspPostInit(&htim8);

}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *timex_hallsensorHandle) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (timex_hallsensorHandle->Instance == TIM5) {
        /* USER CODE BEGIN TIM5_MspInit 0 */

        /* USER CODE END TIM5_MspInit 0 */
        /* TIM5 clock enable */
        __HAL_RCC_TIM5_CLK_ENABLE();

        __HAL_RCC_GPIOH_CLK_ENABLE();
        /**TIM5 GPIO Configuration
        PH10     ------> TIM5_CH1
        PH11     ------> TIM5_CH2
        PH12     ------> TIM5_CH3
        */
        GPIO_InitStruct.Pin = HALL_INPUTU_Pin | HALL_INPUTV_Pin | HALL_INPUTW_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

        /* TIM5 interrupt Init */
        HAL_NVIC_SetPriority(TIM5_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(TIM5_IRQn);
        /* USER CODE BEGIN TIM5_MspInit 1 */

        /* USER CODE END TIM5_MspInit 1 */
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *tim_pwmHandle) {

    if (tim_pwmHandle->Instance == TIM8) {
        /* USER CODE BEGIN TIM8_MspInit 0 */

        /* USER CODE END TIM8_MspInit 0 */
        /* TIM8 clock enable */
        __HAL_RCC_TIM8_CLK_ENABLE();
        /* USER CODE BEGIN TIM8_MspInit 1 */

        /* USER CODE END TIM8_MspInit 1 */
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (timHandle->Instance == TIM8) {
        /* USER CODE BEGIN TIM8_MspPostInit 0 */

        /* USER CODE END TIM8_MspPostInit 0 */

        __HAL_RCC_GPIOI_CLK_ENABLE();

        __HAL_RCC_GPIOH_CLK_ENABLE();
        /**TIM8 GPIO Configuration
        PI5     ------> TIM8_CH1
        PI6     ------> TIM8_CH2
        PI7     ------> TIM8_CH3
        */
        GPIO_InitStruct.Pin = MOTOR_OCPWM1_Pin | MOTOR_OCPWM2_Pin | MOTOR_OCPWM3_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = MOTOR_OCNPWM1_Pin | MOTOR_OCNPWM2_Pin | MOTOR_OCNPWM3_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
        HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
        /* USER CODE BEGIN TIM8_MspPostInit 1 */
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);

        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);

        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

        /* USER CODE END TIM8_MspPostInit 1 */
    }

}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *timex_hallsensorHandle) {

    if (timex_hallsensorHandle->Instance == TIM5) {
        /* USER CODE BEGIN TIM5_MspDeInit 0 */

        /* USER CODE END TIM5_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM5_CLK_DISABLE();

        /**TIM5 GPIO Configuration
        PH10     ------> TIM5_CH1
        PH11     ------> TIM5_CH2
        PH12     ------> TIM5_CH3
        */
        HAL_GPIO_DeInit(GPIOH, HALL_INPUTU_Pin | HALL_INPUTV_Pin | HALL_INPUTW_Pin);

        /* TIM5 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM5_IRQn);
        /* USER CODE BEGIN TIM5_MspDeInit 1 */

        /* USER CODE END TIM5_MspDeInit 1 */
    }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *tim_pwmHandle) {

    if (tim_pwmHandle->Instance == TIM8) {
        /* USER CODE BEGIN TIM8_MspDeInit 0 */

        /* USER CODE END TIM8_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM8_CLK_DISABLE();
        /* USER CODE BEGIN TIM8_MspDeInit 1 */

        /* USER CODE END TIM8_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */
void stop_pwm_output(void) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PWM_TIM_PULSE >> 1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PWM_TIM_PULSE >> 1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, PWM_TIM_PULSE >> 1);
}

void set_pwm_pulse(uint16_t pulse) {
    bldcm_pulse = pulse;
}

void hall_enable(void) {
    __HAL_TIM_SetCounter(&htim5,0);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
}

void hall_disable(void) {
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_TRIGGER);
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);
    HAL_TIMEx_HallSensor_Stop(&htim5);
}

u8 hall_read_temp;
float hall_angle;
float hall_angle_add;
float hall_speed;
int update = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    float temp = (float) HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    hall_angle_add = (float) HALL_ANGLE_FACTOR / (float) (temp);
    hall_speed = (float) HALL_SPEED_FACTOR / (float) (temp);
    hall_read_temp = HAL_GPIO_ReadPin(HALL_INPUTU_GPIO_Port, HALL_INPUTU_Pin);
    hall_read_temp |= HAL_GPIO_ReadPin(HALL_INPUTV_GPIO_Port, HALL_INPUTV_Pin) << 1;
    hall_read_temp |= HAL_GPIO_ReadPin(HALL_INPUTW_GPIO_Port, HALL_INPUTW_Pin) << 2;

    if (hall_read_temp == 0x05) {
        hall_angle = 0.0f + PHASE_SHIFT_ANGLE;
    } else if (hall_read_temp == 0x04) {
        hall_angle = (PI / 3.0f) + PHASE_SHIFT_ANGLE;
    } else if (hall_read_temp == 0x06) {
        hall_angle = (PI * 2.0f / 3.0f) + PHASE_SHIFT_ANGLE;
    } else if (hall_read_temp == 0x02) {
        hall_angle = PI + PHASE_SHIFT_ANGLE;
    } else if (hall_read_temp == 0x03) {
        hall_angle = (PI * 4.0f / 3.0f) + PHASE_SHIFT_ANGLE;
    } else if (hall_read_temp == 0x01) {
        hall_angle = (PI * 5.0f / 3.0f) + PHASE_SHIFT_ANGLE;
    }
    if (hall_angle < 0.0f) {
        hall_angle += 2.0f * PI;
    } else if (hall_angle > (2.0f * PI)) {
        hall_angle -= 2.0f * PI;
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {


}
/* USER CODE END 1 */
