/**
  ******************************************************************************
  * @file    bsp_debug_usart.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   使用串口1，重定向c库printf函数到usart端口，中断接收模式
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 STM32 F103 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */

#include "bsp_debug_usart.h"

UART_HandleTypeDef husart1;
//extern uint8_t ucTemp;  

/**
 * @brief  DEBUG_USART GPIO 配置,工作模式配置。115200 8-N-1
 * @param  无
 * @retval 无
 */
void DEBUG_USART_Config(void) {

    husart1.Instance = DEBUG_USART;

    husart1.Init.BaudRate = DEBUG_USART_BAUDRATE;
    husart1.Init.WordLength = UART_WORDLENGTH_8B;
    husart1.Init.StopBits = UART_STOPBITS_1;
    husart1.Init.Parity = UART_PARITY_NONE;
    husart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    husart1.Init.Mode = UART_MODE_TX_RX;

    HAL_UART_Init(&husart1);

    /*使能串口接收断 */
    //__HAL_UART_ENABLE_IT(&husart1,UART_IT_RXNE);
}


/**
  * @brief UART MSP 初始化 
  * @param huart: UART handle
  * @retval 无
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    GPIO_InitTypeDef GPIO_InitStruct;

    DEBUG_USART_CLK_ENABLE();

    DEBUG_USART_RX_GPIO_CLK_ENABLE();
    DEBUG_USART_TX_GPIO_CLK_ENABLE();

/**USART1 GPIO Configuration    
  PA9     ------> USART1_TX
  PA10    ------> USART1_RX 
  */
    /* 配置Tx引脚为复用功能  */
    GPIO_InitStruct.Pin = DEBUG_USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStruct);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStruct.Pin = DEBUG_USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;    //模式要设置为复用输入模式！
    HAL_GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStruct);

    //HAL_NVIC_SetPriority(DEBUG_USART_IRQ ,0,1);	//抢占优先级0，子优先级1
    // HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ );		    //使能USART1中断通道
}


/*****************  发送字符串 **********************/
void Usart_SendString(uint8_t *str) {
    unsigned int k = 0;
    do {
        HAL_UART_Transmit(&husart1, (uint8_t *) (str + k), 1, 1000);
        k++;
    } while (*(str + k) != '\0');

}

//重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
__attribute__((weak)) int fputc(int ch, FILE *f) {
    /* 发送一个字节数据到串口DEBUG_USART */
    HAL_UART_Transmit(&husart1, (uint8_t *) &ch, 1, 1000);

    return (ch);
}

//重定向c库函数scanf到串口DEBUG_USART，重写向后可使用scanf、getchar等函数
__attribute__((weak)) int fgetc(FILE *f) {
    int ch;
    HAL_UART_Receive(&husart1, (uint8_t *) &ch, 1, 1000);
    return (ch);
}


__attribute__((weak)) int _write(int file, char *ptr, int len){
    /* 发送一个字节数据到串口DEBUG_USART */

    if (HAL_UART_Transmit(&husart1, (uint8_t *) ptr, len, 0xffff) != HAL_OK) {
        Error_Handler();
    }
    return len;
}

/*********************************************END OF FILE**********************/
