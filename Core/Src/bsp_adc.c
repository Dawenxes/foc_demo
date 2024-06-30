/**
  ******************************************************************************
  * @file    bsp_adc.c
  * @author  long
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   NTC驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F407 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */

#include "bsp_adc.h"


DMA_HandleTypeDef DMA_Init_Handle;
ADC_HandleTypeDef adc3;
enum FOC_STATE foc_state = MOTOR_STOP;

static int16_t adc_buff[ADC_NUM_MAX];    // 电压采集缓冲区
static uint32_t adc_mean_t = 0;        // 平均值累加

static uint32_t adc_offset_u = 0;        // 偏执电压
static uint32_t adc_offset_v = 0;        // 偏执电压
static uint32_t adc_offset_w = 0;        // 偏执电压
static SeqQueue adc_u_queue;
static SeqQueue adc_w_queue;
static SeqQueue adc_v_queue;
static SeqQueue adc_vbus_queue;


float Ia, Ib, Ic;
float Vbus;
uint16_t i = 0;
u8 speed_close_loop_flag;
float Iq_ref;
float EKF_Hz;

float theta_add;
float theta;

extern float Rs;
extern float Ls;
extern float flux;

/**
  * @brief  ADC 通道引脚初始化
  * @param  无
  * @retval 无
  */
void ADC_GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    // 使能 GPIO 时钟
    TEMP_ADC_GPIO_CLK_ENABLE();
    VBUS_GPIO_CLK_ENABLE();
    CURR_U_ADC_GPIO_CLK_ENABLE();
    CURR_V_ADC_GPIO_CLK_ENABLE();
    CURR_W_ADC_GPIO_CLK_ENABLE();
    // 配置 IO
    GPIO_InitStructure.Pin = TEMP_ADC_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL; //不上拉不下拉
    HAL_GPIO_Init(TEMP_ADC_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = VBUS_GPIO_PIN;
    HAL_GPIO_Init(VBUS_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = CURR_U_ADC_GPIO_PIN;
    HAL_GPIO_Init(CURR_U_ADC_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = CURR_V_ADC_GPIO_PIN;
    HAL_GPIO_Init(CURR_V_ADC_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = CURR_W_ADC_GPIO_PIN;
    HAL_GPIO_Init(CURR_W_ADC_GPIO_PORT, &GPIO_InitStructure);
}

//void adc_dma_init(void) {
//    // ------------------DMA Init 结构体参数 初始化--------------------------
//    // ADC3使用DMA2，数据流0，通道0，这个是手册固定死的
//    // 开启DMA时钟
//    TEMP_ADC_DMA_CLK_ENABLE();
//    // 数据传输通道
//    DMA_Init_Handle.Instance = TEMP_ADC_DMA_STREAM;
//    // 数据传输方向为外设到存储器
//    DMA_Init_Handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
//    // 外设寄存器只有一个，地址不用递增
//    DMA_Init_Handle.Init.PeriphInc = DMA_PINC_DISABLE;
//    // 存储器地址固定
//    DMA_Init_Handle.Init.MemInc = DMA_MINC_ENABLE;
//    // 外设数据大小为半字，即两个字节
//    DMA_Init_Handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
//    //	存储器数据大小也为半字，跟外设数据大小相同
//    DMA_Init_Handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
//    // 循环传输模式
//    DMA_Init_Handle.Init.Mode = DMA_CIRCULAR;
//    // DMA 传输通道优先级为高，当使用一个DMA通道时，优先级设置不影响
//    DMA_Init_Handle.Init.Priority = DMA_PRIORITY_HIGH;
//    // 禁止DMA FIFO	，使用直连模式
//    DMA_Init_Handle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//    // FIFO 大小，FIFO模式禁止时，这个不用配置
//    DMA_Init_Handle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
//    DMA_Init_Handle.Init.MemBurst = DMA_MBURST_SINGLE;
//    DMA_Init_Handle.Init.PeriphBurst = DMA_PBURST_SINGLE;
//    // 选择 DMA 通道，通道存在于流中
//    DMA_Init_Handle.Init.Channel = TEMP_ADC_DMA_CHANNEL;
//    //初始化DMA流，流相当于一个大的管道，管道里面有很多通道
//    HAL_DMA_Init(&DMA_Init_Handle);
//
//    __HAL_LINKDMA(&adc3, DMA_Handle, DMA_Init_Handle);
//}

/**
  * @brief  ADC 模式配置,时钟84M, ADC时钟为84M/4=21M, 3+12=15个周期=
  * @param  无
  * @retval 无
  */
void ADC_Mode_Config(void) {
    // 开启ADC时钟
    TEMP_ADC_CLK_ENABLE();
    // -------------------ADC Init 结构体 参数 初始化------------------------
    // ADC3
    adc3.Instance = TEMP_ADC;
    // 时钟为fpclk 4分频	
    adc3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
    // ADC 分辨率
    adc3.Init.Resolution = ADC_RESOLUTION_12B;
    // 扫描模式，多通道采集才需要	
    adc3.Init.ScanConvMode = ENABLE;
    // 连续转换	
    adc3.Init.ContinuousConvMode = DISABLE;
    // 非连续转换	
    adc3.Init.DiscontinuousConvMode = DISABLE;
    // 非连续转换个数
    adc3.Init.NbrOfDiscConversion = 0;
    //禁止外部边沿触发    
    adc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    //使用软件触发
    adc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    //数据右对齐	
    adc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    //转换通道 5个
    adc3.Init.NbrOfConversion = 0;
    //使能连续转换请求
    adc3.Init.DMAContinuousRequests = DISABLE;
    //转换完成标志
    adc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    // 初始化ADC	                          
    if (HAL_ADC_Init(&adc3) != HAL_OK) {
        while (1);
    }

    //---------------------------------------------------------------------------
    // 配置 ADC 通道转换顺序为1，第一个转换，采样时间为3个时钟周期
    ADC_InjectionConfTypeDef injectionConfTypeDef;
    injectionConfTypeDef.InjectedChannel = CURR_U_ADC_CHANNEL;
    injectionConfTypeDef.InjectedRank = ADC_INJECTED_RANK_1;
    injectionConfTypeDef.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
    injectionConfTypeDef.InjectedOffset = 0;
    injectionConfTypeDef.InjectedNbrOfConversion = 4;
    injectionConfTypeDef.AutoInjectedConv = DISABLE;
    injectionConfTypeDef.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T8_CC4;
    injectionConfTypeDef.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
    injectionConfTypeDef.InjectedDiscontinuousConvMode = DISABLE;

    if (HAL_ADCEx_InjectedConfigChannel(&adc3, &injectionConfTypeDef) != HAL_OK) {
        while (1);
    }

    injectionConfTypeDef.InjectedChannel = CURR_V_ADC_CHANNEL;
    injectionConfTypeDef.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&adc3, &injectionConfTypeDef) != HAL_OK) {
        while (1);
    }
    injectionConfTypeDef.InjectedChannel = CURR_W_ADC_CHANNEL;
    injectionConfTypeDef.InjectedRank = ADC_INJECTED_RANK_3;
    if (HAL_ADCEx_InjectedConfigChannel(&adc3, &injectionConfTypeDef) != HAL_OK) {
        while (1);
    }
    injectionConfTypeDef.InjectedChannel = VBUS_ADC_CHANNEL;
    injectionConfTypeDef.InjectedRank = ADC_INJECTED_RANK_4;
    if (HAL_ADCEx_InjectedConfigChannel(&adc3, &injectionConfTypeDef) != HAL_OK) {
        while (1);
    }
    HAL_ADCEx_InjectedStart_IT(&adc3);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle) {
    if (adcHandle->Instance == ADC3) {
        __HAL_RCC_ADC3_CLK_ENABLE();

        HAL_NVIC_SetPriority(ADC_VBUS_IRQ, 1, 0);
        HAL_NVIC_EnableIRQ(ADC_VBUS_IRQ);
    }
}

/**
  * @brief  ADC 采集初始化
  * @param  无
  * @retval 无
  */
void ADC_Init(void) {
    InitQueue(&adc_u_queue);
    InitQueue(&adc_w_queue);
    InitQueue(&adc_v_queue);
    InitQueue(&adc_vbus_queue);

    ADC_GPIO_Config();
    ADC_Mode_Config();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
}

/**
  * @brief  常规转换在非阻塞模式下完成回调
  * @param  hadc: ADC  句柄.
  * @retval 无
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    static uint16_t flag = 0;

    /* 计算电流通道采样的平均值 */
    EnQueue(&adc_u_queue, HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1));
    EnQueue(&adc_v_queue, HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2));
    EnQueue(&adc_w_queue, HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3));
    EnQueue(&adc_vbus_queue, HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4));

    /* 计算温度通道采样的平均值 */
//    for (uint32_t count = 4; count < ADC_NUM_MAX; count += 5) {
//        adc_mean += (int32_t) adc_buff[count];
//    }
//    adc_mean_t = adc_mean / (ADC_NUM_MAX_COUNT);    // 保存平均值
//    adc_mean = 0;

    if (foc_state == MOTOR_RUN) {
        motor_start();
        foc_state = MOTOR_RUNNING;
    } else if (foc_state == MOTOR_RUNNING) {
        motor_run();
    } else {
        if (foc_state == MOTOR_GET_OFFSET && flag < 128) {
            flag++;
        } else if (foc_state == MOTOR_GET_OFFSET && flag >= 128) {
            adc_offset_u = mean(&adc_u_queue);
            adc_offset_v = mean(&adc_v_queue);
            adc_offset_w = mean(&adc_w_queue);
            foc_state = MOTOR_RUN;
        }
    }
}

/**
  * @brief  获取温度传感器端的电压值
  * @param  无
  * @retval 转换得到的电压值
  */
float get_ntc_v_val(void) {
    float vdc = GET_ADC_VDC_VAL(adc_mean_t);      // 获取电压值

    return vdc;
}

/**
  * @brief  获取温度传感器端的电阻值
  * @param  无
  * @retval 转换得到的电阻值
  */
float get_ntc_r_val(void) {
    float r = 0;
    float vdc = get_ntc_v_val();
    r = (VREF - vdc) / (vdc / (float) 4700.0);
    return r;
}

/**
  * @brief  获取温度传感器的温度
  * @param  无
  * @retval 转换得到的温度，单位：（℃）
  */
float get_ntc_t_val(void) {
    float t = 0;             // 测量温度
    float Rt = 0;            // 测量电阻
    float Ka = 273.15;       // 0℃ 时对应的温度（开尔文）
    float R25 = 10000.0;     // 25℃ 电阻值
    float T25 = Ka + 25;     // 25℃ 时对应的温度（开尔文）
    float B = 3950.0;        /* B-常数：B = ln(R25 / Rt) / (1 / T – 1 / T25)，
                             其中 T = 25 + 273.15 */

    Rt = get_ntc_r_val();    // 获取当前电阻值

    t = B * T25 / (B + log(Rt / R25) * T25) - Ka;    // 使用公式计算

    return t;
}

/**
  * @brief  获取V相的电流值
  * @param  无
  * @retval 转换得到的电流值
  */
int32_t get_curr_val_v(void) {
    int32_t curr_adc_mean = 0;         // 电流 ACD 采样结果平均值

    curr_adc_mean = mean(&adc_v_queue);    // 保存平均值

    curr_adc_mean -= adc_offset_v;                     // 减去偏置电压

    float vdc = GET_ADC_VDC_VAL(curr_adc_mean);      // 获取电压值

    return GET_ADC_CURR_VAL(vdc);
}

/**
  * @brief  获取U相的电流值
  * @param  无
  * @retval 转换得到的电流值
  */
int32_t get_curr_val_u(void) {
    int32_t curr_adc_mean = 0;         // 电流 ACD 采样结果平均值

    curr_adc_mean = mean(&adc_u_queue);    // 保存平均值

    curr_adc_mean -= adc_offset_u;                     // 减去偏置电压

    float vdc = GET_ADC_VDC_VAL(curr_adc_mean);      // 获取电压值

    return GET_ADC_CURR_VAL(vdc);
}

/**
  * @brief  获取W相的电流值
  * @param  无
  * @retval 转换得到的电流值
  */
int32_t get_curr_val_w(void) {
    int32_t curr_adc_mean = 0;         // 电流 ACD 采样结果平均值

    curr_adc_mean = mean(&adc_w_queue);

    curr_adc_mean -= adc_offset_w;                     // 减去偏置电压

    float vdc = GET_ADC_VDC_VAL(curr_adc_mean);      // 获取电压值

    return GET_ADC_CURR_VAL(vdc);
}

/**
  * @brief  获取电源电压值
  * @param  无
  * @retval 转换得到的电压值
  */
float get_vbus_val(void) {
    float vdc = GET_ADC_VDC_VAL(mean(&adc_vbus_queue));      // 获取电压值
    return GET_VBUS_VAL(vdc);
}


void motor_run(void) {
    hall_angle += hall_angle_add;
    if (hall_angle < 0.0f) {
        hall_angle += 2.0f * PI;
    } else if (hall_angle > (2.0f * PI)) {
        hall_angle -= 2.0f * PI;
    }
    Vbus = get_vbus_val();
    Ia = get_curr_val_u();
    Ib = get_curr_val_v();
    Ic = -Ia - Ib;

    if (speed_close_loop_flag == 0) {
        if ((Iq_ref < MOTOR_STARTUP_CURRENT)) {
            Iq_ref += 0.00003f;
        } else {
            speed_close_loop_flag = 1;
        }
    } else {
        if (speed_close_loop_flag == 1) {
            if (Iq_ref > (MOTOR_STARTUP_CURRENT / 2.0f)) {
                Iq_ref -= 0.001f;
            } else {
                speed_close_loop_flag = 2;
            }
        }
    }

#ifdef  HALL_FOC_SELECT
    if ((hall_speed * 2.0f * PI) > SPEED_LOOP_CLOSE_RAD_S) {
        FOC_Input.Id_ref = 0.0f;
        Speed_Fdk = hall_speed * 2.0f * PI;
        FOC_Input.Iq_ref = Speed_Pid_Out;
    } else {
        FOC_Input.Id_ref = 0.0f;
        FOC_Input.Iq_ref = Iq_ref;
        Speed_Pid.I_Sum = Iq_ref;
    }


    FOC_Input.theta = hall_angle;
    FOC_Input.speed_fdk = hall_speed * 2.0f * PI;
#endif
    FOC_Input.Tpwm = PWM_TIM_PULSE_TPWM;
    FOC_Input.Udc = Vbus;
    FOC_Input.flux = flux;
    FOC_Input.Rs = Rs;
    FOC_Input.Ls = Ls;
    FOC_Input.ia = ((float) Ia) / 1000.0f;
    FOC_Input.ib = ((float) Ib) / 1000.0f;
    FOC_Input.ic = ((float) Ic) / 1000.0f;
    foc_algorithm_step();

    if (foc_state == MOTOR_RUN || foc_state == MOTOR_RUNNING) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (u16) (FOC_Output.Tcmp1));
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (u16) (FOC_Output.Tcmp2));
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (u16) (FOC_Output.Tcmp3));
    } else {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PWM_TIM_PULSE >> 1);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PWM_TIM_PULSE >> 1);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, PWM_TIM_PULSE >> 1);
    }

}


void motor_start(void) {
    foc_algorithm_initialize();
    Speed_Ref = 20.0F;
    speed_close_loop_flag = 0;
    Iq_ref = 0.0f;

    hall_angle_add = 0.0005f;
    hall_speed = 0.0f;
    set_bldcm_enable();
}
/*********************************** END OF FILE *********************************************/
