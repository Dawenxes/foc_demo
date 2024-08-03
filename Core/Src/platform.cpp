// by pengyao1207
// in May 2023 

#include "platform.h"

#include "main.h"
#include "adc.h"
#include "motor.h"
#include "uart.h"
#include "pwm.h"
#include "timer.h"
#include "motor.h"
#include "hallsensor.h"
#include "smosensor.h"
#include "hallabzsensor.h"
#include "control.h"
#include "uart.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

static uint8_t sg_rx4 = 0;
static uint8_t sg_rx2 = 0;
static uint8_t sg_rx3 = 0;

// STM32有堆内存，但是如果使用堆内存的话，就没法在STM32CubeMonitor里面监视了

// 构造时间轴（频率，读取方法）
static Timer timer(84000000,[]()-> uint32_t{return htim2.Instance->CNT;});

// 构造霍尔对象0（时间轴，读取方法）
//static HallSensor hall0(&timer,[]()->uint8_t{
//	uint8_t A_active = HAL_GPIO_ReadPin(HALL0_A_GPIO_Port,HALL0_A_Pin);
//	uint8_t B_active = HAL_GPIO_ReadPin(HALL0_B_GPIO_Port,HALL0_B_Pin);
//	uint8_t C_active = HAL_GPIO_ReadPin(HALL0_C_GPIO_Port,HALL0_C_Pin);
//	uint8_t hall0_state = C_active + (B_active << 1) + (A_active << 2);
//	return hall0_state;
//});

// 构造霍尔ABZ对象0（时间轴，转360度电角度在AB线上产生的中断数之和，HALL读取方法，ABZ读取方法）
static HallAbzSensor hallabz0(&timer,2000,[]()->uint8_t{
	uint8_t A_active = HAL_GPIO_ReadPin(HALL0_A_GPIO_Port,HALL0_A_Pin);
	uint8_t B_active = HAL_GPIO_ReadPin(HALL0_B_GPIO_Port,HALL0_B_Pin);
	uint8_t C_active = HAL_GPIO_ReadPin(HALL0_C_GPIO_Port,HALL0_C_Pin);
	uint8_t hall0_state = C_active + (B_active << 1) + (A_active << 2);
	return hall0_state;
},[]()-> uint8_t {
	uint8_t A = HAL_GPIO_ReadPin(ENCODE_A_GPIO_Port,ENCODE_A_Pin);
	uint8_t B = HAL_GPIO_ReadPin(ENCODE_B_GPIO_Port,ENCODE_B_Pin);
	uint8_t Z = 0; // 不使用Z信号
	uint8_t state = (A << 2) | (B << 1) | Z;
	return state;
});

// 滑膜目前有点问题
//static SmoSensor smo0(&timer,[]()->uint8_t{
//	uint8_t A_active= HAL_GPIO_ReadPin(HALL0_A_GPIO_Port,HALL0_A_Pin);
//	uint8_t B_active= HAL_GPIO_ReadPin(HALL0_B_GPIO_Port,HALL0_B_Pin);
//	uint8_t C_active= HAL_GPIO_ReadPin(HALL0_C_GPIO_Port,HALL0_C_Pin);
//	uint8_t hall1_state = C_active + (B_active << 1) + (A_active << 2);
//	return hall1_state;
//});

// 构造传感器对象0
// 注意cpp中只有引用和指针可以触发虚表查找，我以前一直认为查表性能消耗大，但这里实测下来，5ns以内，忽略不计
//static SensorBase & sensor0 = smo0;
static SensorBase & sensor0 = hallabz0; 


// 构造霍尔对象1
static HallSensor hall1(&timer,[]()->uint8_t{
	uint8_t A_active= HAL_GPIO_ReadPin(HALL1_A_GPIO_Port,HALL1_A_Pin);
	uint8_t B_active= HAL_GPIO_ReadPin(HALL1_B_GPIO_Port,HALL1_B_Pin);
	uint8_t C_active= HAL_GPIO_ReadPin(HALL1_C_GPIO_Port,HALL1_C_Pin);
	uint8_t hall1_state = C_active + (B_active << 1) + (A_active << 2);
	return hall1_state;
});

// 构造传感器对象1
static SensorBase & sensor1 = hall1;

// 构造ADC对象0（读取位置，读取方法）
static Adc adc0(Adc::Mode::VW,[](bool is_cai,float * a1,float * a2,float * a3){
	uint16_t v,w;
	if(is_cai) // 校准模式，使用间断轮询来读值,一次轮询读值用时30us，主要时间用在开启ADC上，估计是Hal库的bug
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);
		v = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);
		w = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
	}
	else // 正常模式，使用ADC注入来读值，这里几乎不用花费时间
	{
		v = hadc1.Instance->JDR1;
		w = hadc1.Instance->JDR2;
	}
	// 下面的50是电流增益（这个乘法不知道为什么用时达到1us以上....指令流水断了？？）
	(*a1) = v / 4096.f * 3.3f * 50;
	(*a2) = w / 4096.f * 3.3f * 50;
});


// 构造ADC对象1
static Adc adc1(Adc::Mode::VW,[](bool is_cai,float * a1,float * a2,float * a3)
{
	uint16_t v,w;
	if(is_cai)
	{
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2,HAL_MAX_DELAY);
		v = HAL_ADC_GetValue(&hadc2);
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2,HAL_MAX_DELAY);
		w = HAL_ADC_GetValue(&hadc2);
		HAL_ADC_Stop(&hadc2);
	}
	else
	{
		v = hadc2.Instance->JDR1;
		w = hadc2.Instance->JDR2;
	}
	(*a1) = v / 4096.f * 3.3f * 50;
	(*a2) = w / 4096.f * 3.3f * 50;
});

// 构造PWM对象（频率，半周期计数值，设置方法）
static Pwm pwm0(12000,1750,[](uint32_t ccr1,uint32_t ccr2,uint32_t ccr3){
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,ccr1);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,ccr2);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,ccr3);
});
static Pwm pwm1(12000,1750,[](uint32_t ccr1,uint32_t ccr2,uint32_t ccr3){
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,ccr1);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,ccr2);
	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,ccr3);
});

// 构造电机对象（极对数，最大电压，最大电流，传感器，pwm，电流采样）
static Motor motor0(2,24,5,8000,&sensor0,&pwm0,&adc0);
static Motor motor1(2,24,5,8000,&sensor1,&pwm1,&adc1);


// 构造控制对象
static Control control0(&motor0);
static Control control1(&motor1);

// 构造串口对象
static UartConn uart_conn(&control0,&control1);

// 这个函数应该在main函数里面被调用
extern "C" void platform_init()
{

	HAL_TIM_Base_Start_IT(&htim2);
	
	// 设置霍尔偏移，某些摔过的电机霍尔安装可能是歪的，正常电机不需要这个
	hallabz0.set_offset(-114);

	// 要先把下半桥的mos管打开(本来就是打开的...这里只是为了保险一点)，才能进行adc校准
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);


	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);


	HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_3);

	// 要进行一点点延时，确保电流稳定
	HAL_Delay(2000);

	// ADC校准
	adc0.cai_adc();
	adc1.cai_adc();
	
	// 开启ADC注入
	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_ADCEx_InjectedStart(&hadc2);

	// 打开注入用的PWM通道
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

	// 开启PWM更新中断
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);

	// 串口接收，目前只用到了串口4
	HAL_UART_Receive_IT(&huart4,&sg_rx4,1);
	HAL_UART_Receive_IT(&huart2,&sg_rx2,1);
	HAL_UART_Receive_IT(&huart3,&sg_rx3,1);

	while (1)
	{
		HAL_GPIO_TogglePin(RUN_LED_GPIO_Port, RUN_LED_Pin);
		HAL_Delay(100);
	}
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	if(htim->Instance == TIM1)
	{
		control0.update(); // 更新控制器状态，此处耗时约10us，其中adc读值1us左右，更新电机状态用时4us，pid计算用时2us，svpwm计算用时3us
	}
	else if(htim->Instance == TIM8)
	{
		control1.update();
	}
	else if(htim->Instance == TIM2)
	{
		timer.update(); // 更新时间轴
	}
}

uint32_t g_encode_num = 0;
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == HALL0_A_Pin || GPIO_Pin == HALL0_B_Pin || GPIO_Pin == HALL0_C_Pin)
	{
		hallabz0.update_hall(); // 更新传感器状态，用时<1us，我没有精确去测量它
	}
	else if(GPIO_Pin == HALL1_A_Pin || GPIO_Pin == HALL1_B_Pin || GPIO_Pin == HALL1_C_Pin)
	{
		hall1.update();
	}
	else if(GPIO_Pin == ENCODE_A_Pin)
	{
		hallabz0.update_abz('A');
	}
	else if(GPIO_Pin == ENCODE_B_Pin)
	{
		hallabz0.update_abz('B');
	}
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART4)
	{
		if(__HAL_UART_GET_FLAG(huart,UART_FLAG_ORE) == SET)
		{
			__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_ORE);
			// STM32的串口是全双工的，加锁是完全没必要的，但是Hal库把它当做半双工处理了
			// 所以这里设置失败，完全可以强制解锁
			while(HAL_UART_Receive_IT(huart, &sg_rx4, 1) == HAL_BUSY)
			{
				__HAL_UNLOCK(huart);
			}
		}
	}
	else if(huart->Instance == USART2)
	{
		if(__HAL_UART_GET_FLAG(huart,UART_FLAG_ORE) == SET)
		{
			__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_ORE);
			while(HAL_UART_Receive_IT(huart, &sg_rx2, 1) == HAL_BUSY)
			{
				__HAL_UNLOCK(huart);
			}
		}
	}
	else if(huart->Instance == USART3)
	{
		if(__HAL_UART_GET_FLAG(huart,UART_FLAG_ORE) == SET)
		{
			__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_ORE);
			while(HAL_UART_Receive_IT(huart, &sg_rx3, 1) == HAL_BUSY)
			{
				__HAL_UNLOCK(huart);
			}
		}
	}

}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == UART4)
	{
		// 处理通讯，没什么耗时，但是要确保串口的优先级高于传感器和PWM更新中断的优先级，否则会漏消息！！！！！
		// 毕竟，如果控制两个电机的话，cpu有1/4的时间都在PWM更新中断里面......
		uart_conn.update(sg_rx4); 
		while(HAL_UART_Receive_IT(huart, &sg_rx4,1) == HAL_BUSY)
		{
			__HAL_UNLOCK(huart);
		}
	}
	else if(huart->Instance == USART2)
	{
 		while(HAL_UART_Receive_IT(huart, &sg_rx2,1) == HAL_BUSY)
		{
			__HAL_UNLOCK(huart);
		}
	}
	else if(huart->Instance == USART3)
	{
 		while(HAL_UART_Receive_IT(huart, &sg_rx3,1) == HAL_BUSY)
		{
			__HAL_UNLOCK(huart);
		}
	}
}

