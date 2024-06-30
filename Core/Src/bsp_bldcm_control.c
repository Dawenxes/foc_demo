/**
  ******************************************************************************
  * @file    bsp_bldcm_control.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   ��ˢ������ƽӿ�
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 F407 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */

#include "bsp_bldcm_control.h"

/* ˽�б��� */
static bldcm_data_t bldcm_data;

/* �ֲ����� */
static void sd_gpio_config(void);

/**
  * @brief  �����ʼ��
  * @param  ��
  * @retval ��
  */
void bldcm_init(void) {
    MX_TIM8_Init();
    MX_TIM5_Init();
    sd_gpio_config();        // sd ���ų�ʼ��
}

/**
  * @brief  ��� SD �������ų�ʼ��
  * @param  ��
  * @retval ��
  */
static void sd_gpio_config(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    /* ��ʱ��ͨ���������Ŷ˿�ʱ��ʹ�� */
    SHUTDOWN_GPIO_CLK_ENABLE();

    /* ����IO��ʼ�� */
    /*�����������*/
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    /*������������ */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    /*ѡ��Ҫ���Ƶ�GPIO����*/
    GPIO_InitStruct.Pin = SHUTDOWN_PIN;

    /*���ÿ⺯����ʹ���������õ�GPIO_InitStructure��ʼ��GPIO*/
    HAL_GPIO_Init(SHUTDOWN_GPIO_PORT, &GPIO_InitStruct);
}




/**
  * @brief  ʹ�ܵ��
  * @param  ��
  * @retval ��
  */
void set_bldcm_enable(void) {
    BLDCM_ENABLE_SD();
    HAL_Delay(1);
    hall_enable();
}



/**
  * @brief  ���õ��
  * @param  ��
  * @retval ��
  */
void set_bldcm_disable(void) {
    /* ���û����������ӿ� */
    hall_disable();

    /* ֹͣ PWM ��� */
    stop_pwm_output();

    /* �ر� MOS �� */
    BLDCM_DISABLE_SD();
}

/*********************************************END OF FILE**********************/
