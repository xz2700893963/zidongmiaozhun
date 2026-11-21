#include "bsp_usart.h"
#include <stdio.h>
#include <stdarg.h>
#include "stm32f4xx.h"


void uart1_init(void)
{
	// 定义一个GPIO初始化结构体
	GPIO_InitTypeDef GPIO_InitStructure;
	// 定义一个USART初始化结构体
	USART_InitTypeDef USART_InitStructure;
	// 定义一个NVIC初始化结构体
	NVIC_InitTypeDef NVIC_InitStructure;

	// 使能GPIOA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// 使能UART1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	// ----------------- UART1 引脚配置 -----------------
	// 将GPIOA的Pin9配置为USART1发送复用功能
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	// 将GPIOA的Pin10配置为USART1接收复用功能
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	// 配置GPIO引脚为9、10，PA9(TX)、PA10(RX)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	// 配置GPIO模式为复用功能模式，用于USART
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	// 配置GPIO速度为50MHz
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// 配置GPIO输出类型为推挽输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	// 配置GPIO上拉/下拉为上拉
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	// 初始化GPIOA
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// ----------------- UART1 参数初始化 -----------------

	// 设置波特率为115200
	USART_InitStructure.USART_BaudRate = 115200;
	// 数据格式为8位数据长度
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 1个停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 不使用校验位
	USART_InitStructure.USART_Parity = USART_Parity_No;
	// 不使用硬件流控
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置USART为收发模式
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 初始化UART1
	USART_Init(USART1, &USART_InitStructure);

	// 开启UART1接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);

	// ----------------- UART1 NVIC配置 -----------------

	// 配置NVIC中断通道为UART1中断
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	// 抢占优先级为3
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	// 子优先级为2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	// 使能该中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// 初始化NVIC
	NVIC_Init(&NVIC_InitStructure);

	// 使能UART1
	USART_Cmd(USART1, ENABLE);
}

void uart2_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2, ENABLE);
}

void uart3_init(void)
{
	// 定义一个GPIO初始化结构体
	GPIO_InitTypeDef GPIO_InitStructure;
	// 定义一个USART初始化结构体
	USART_InitTypeDef USART_InitStructure;
	// 定义一个NVIC初始化结构体
	NVIC_InitTypeDef NVIC_InitStructure;

	// 使能GPIOB时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	// 使能UART3时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	// ----------------- UART3 引脚配置 -----------------
	// 将GPIOB的Pin10配置为USART3发送复用功能
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	// 将GPIOB的Pin11配置为USART3接收复用功能
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	// 配置GPIO引脚为10、11，PB10(TX)、PB11(RX)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	// 配置GPIO模式为复用功能模式，用于USART
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	// 配置GPIO速度为50MHz
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// 配置GPIO输出类型为推挽输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	// 配置GPIO上拉/下拉为上拉
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	// 初始化GPIOB
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// ----------------- UART3 参数初始化 -----------------

	// 设置波特率为256000
	USART_InitStructure.USART_BaudRate = 256000;
	// 数据格式为8位数据长度
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 1个停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 不使用校验位
	USART_InitStructure.USART_Parity = USART_Parity_No;
	// 不使用硬件流控
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置USART为收发模式
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 初始化UART3
	USART_Init(USART3, &USART_InitStructure);

	// 开启UART3接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

	// ----------------- UART3 NVIC配置 -----------------

	// 配置NVIC中断通道为UART3中断
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	// 抢占优先级为3
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	// 子优先级为2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	// 使能该中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// 初始化NVIC
	NVIC_Init(&NVIC_InitStructure);

	// 使能UART3
	USART_Cmd(USART3, ENABLE);
}

/**
  * @brief 串口发送一个字节
  * @param Byte 要发送的一个字节
  * @retval 无
  */
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Serial3_SendByte(uint8_t Byte)
{
	USART_SendData(USART3, Byte);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}


/* USART1 is reserved for USB-TTL telemetry and runtime tuning. */
