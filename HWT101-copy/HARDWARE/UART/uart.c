#include "uart.h"
#include "sys.h"
#include <stdio.h>
#include "stdarg.h"    
#include "string.h" 

uint8_t Serial_Flag;

uint8_t Serial_TXPacket[3];
uint8_t Serial_RXPacket[8];

uint8_t Serial2_TXPacket[3];
uint8_t Serial2_RXPacket[8];

uint8_t Serial3_TXPacket[3];
uint8_t Serial3_RXPacket[11];
uint8_t Serial3_HW101_Angle[2];

uint8_t USART2_TX_BUF[100];


// PA9    TX
// PA10   RX
/********************************
函数功能 : 串口1的初始化
输入参数 : 无
输出参数 ；无
*********************************/
void UART1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
  USART_Init(USART1, &USART_InitStructure); 
	
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
  USART_Cmd(USART1, ENABLE);
}
/********************************
函数功能 : 串口1 发送一个字节
输入参数 : 一个字节
输出参数 ；无
*********************************/
void USART1_SendBits(uint8_t data)
{
	USART1->DR = data;
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
}
/********************************
函数功能 : 串口1 发送一个数组
输入参数 : 一个数组
输出参数 ；无
*********************************/
void UART1_SendArray(uint8_t *array,uint8_t length)
{
	uint8_t i;
	for(i=0;i<length;i++)
	{
		USART1_SendBits(array[i]);
	}
}
/********************************
函数功能 : 串口1 发送一个字符串
输入参数 : 一个字符串
输出参数 ；无
*********************************/
void UART1_SendString(uint8_t *String1)
{
	uint8_t i;
	for(i=0;String1[i]!='\0';i++)
	{
		USART1_SendBits(String1[i]);		
	}
}
/********************************
函数功能 : 串口1 发送一个数字
输入参数 : 一个数字
输出参数 ；无
*********************************/
void UART1_SendNum(uint32_t Number,uint8_t Length)
{
	uint8_t i;
	for(i=0;i<Length;i++)
	{
		USART1_SendBits(Number/Serial_Pow(10,Length-i-1)%10 + '0');
	}
}
/********************************
函数功能 : x的y次方
输入参数 : x与y
输出参数 ；无
*********************************/
uint32_t Serial_Pow(uint32_t x,uint32_t y)
{
	uint32_t Result=1;
	while(y--)
	{
		Result *= x;
	}
	return Result;
}
/********************************
函数功能 : 发送一个数据包 针头0xff 针尾0xfe
输入参数 : 数据包
输出参数 ；无
*********************************/
void UART_SendPacket(uint8_t *Serial_TXPacket)
{
	USART1_SendBits(0xFF); //包头
	UART1_SendArray(Serial_TXPacket,3); //数据
	USART1_SendBits(0xFE); //包尾
}
/********************************
函数功能 : 发送一个数据包 针头0xff 针尾0xfe
输入参数 : 一个字节
输出参数 ；无
*********************************/
void UART_SendPacket2UP(uint8_t data)
{
	USART1_SendBits(0xFF); //包头
	USART1_SendBits(data);
	USART1_SendBits(0xFE); //包尾
}
/********************************
函数功能 : 发送一个数据包 针头0xaf 针尾0xae
输入参数 : 一个字节,用于一键启动
输出参数 ；无
*********************************/
void UART_SendPacket3UP(uint8_t data)
{
	USART1_SendBits(0xAF); //包头
	USART1_SendBits(data);
	USART1_SendBits(0xAE); //包尾
}


/********************************
函数功能 : 串口1的中断处理函数
输入参数 : 无
输出参数 ；无
*********************************/
void USART1_IRQHandler(void)
{
	static uint8_t RX1State=0;
	static uint8_t pRx1Packet=0;
	if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET)
	{
		uint8_t Rx1Data = USART_ReceiveData(USART1);
		if(RX1State == 0)
		{
			if(Rx1Data == 0xFF)
			{
					RX1State =1;
					pRx1Packet=0;
			}
		}
		else if(RX1State == 1)
		{
				Serial_RXPacket[pRx1Packet] = Rx1Data;
				pRx1Packet++;
				if(pRx1Packet>7)
				{
					RX1State=2;
				}
		}
		else if(RX1State == 2)
		{
			if(Rx1Data == 0xFE)
			{
					RX1State = 0;
					Serial_Flag = 1;
			}
		}
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
}
/********************************
函数功能 : 串口1的获取接收数据完成的标志位
输入参数 : 无
输出参数 ；1 接收完成  0 接收未完成
*********************************/
uint8_t Serial1_GetRxFlag(void)
{
	if(Serial_Flag == 1)
	{
		Serial_Flag=0;
		return 1;
	}
	return 0;
}

/******************************************************************************
PD5  TX2
PD6  RX2
******************************************************************************/
/********************************
函数功能 : 串口2的初始化
映射
输入参数 : 无
输出参数 ；无
*********************************/
void UART2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
  USART_Init(USART2, &USART_InitStructure); 
	
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
  USART_Cmd(USART2, ENABLE);
}



/********************************
函数功能 : 串口2 发送一个字节
输入参数 : 一个字节
输出参数 ；无
*********************************/
void USART2_SendBits(uint8_t data)
{
	USART2->DR = data;
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
}


int fputc(int ch, FILE *f)
{
	USART2_SendBits(ch);
	
	return ch;
}
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_RXNE) == SET)
	{
		static uint8_t RX2State=0;
	static uint8_t pRx2Packet=0;
	if(USART_GetITStatus(USART2,USART_IT_RXNE) == SET)
	{
		uint8_t Rx2Data = USART_ReceiveData(USART2);
		if(RX2State == 0)
		{
			if(Rx2Data == 0xAF)
			{
					RX2State =1;
					pRx2Packet=0;
			}
		}
		else if(RX2State == 1)
		{
				Serial2_RXPacket[pRx2Packet] = Rx2Data;
				pRx2Packet++;
				if(pRx2Packet>7)
				{
					RX2State=2;
				}
		}
		else if(RX2State == 2)
		{
			if(Rx2Data == 0xAE)
			{
					RX2State = 0;
					Serial_Flag = 1;
			}
		}
			USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	
	}
}
}


void u2_printf(char* fmt,...)  
{  
 u16 i,j;
 va_list ap;
 va_start(ap,fmt);
 vsprintf((char*)USART2_TX_BUF,fmt,ap);
 va_end(ap);
 i=strlen((const char*)USART2_TX_BUF);//此次发送数据的长度
 for(j=0;j<i;j++)//循环发送数据
 { 
  USART_SendData(USART2,(uint8_t)USART2_TX_BUF[j]);   //发送数据到串口2
   while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);  //等待上次传输完成 
 } 
 USART_SendData(USART2,(uint8_t)0xff);  //这个函数改为你的单片机的串口发送单字节函数
 while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); 
 USART_SendData(USART2,(uint8_t)0xff);  //这个函数改为你的单片机的串口发送单字节函数
 while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); 
 USART_SendData(USART2,(uint8_t)0xff);  //这个函数改为你的单片机的串口发送单字节函数
 while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); 
}
/******************************************************************************
PB10  TX3(没有选串口重映射的那个脚)
PB11  RX3(没有选串口重映射的那个脚)
******************************************************************************/
/********************************
函数功能 : 串口3的初始化
映射
输入参数 : 无
输出参数 ；无
*********************************/
void UART3_Init(void)
{
	 
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	 /*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	/*GPIO初始化*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	/*NVIC配置*/
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	/*USART初始化*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长，选择8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//停止位，选择1位
	USART_InitStructure.USART_Parity = USART_Parity_No;//奇偶校验，不需要
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流控制，不需要
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	 //模式，发送模式和接收模式均选择
  USART_Init(USART3, &USART_InitStructure); 
	
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
  USART_Cmd(USART3, ENABLE);
}
/********************************
函数功能 : 串口3的中断处理函数
输入参数 : 无
输出参数 ；无
*********************************/
void USART3_IRQHandler(void)
{
	static uint8_t RX3State=0;
	static uint8_t pRx3Packet=0;
	if(USART_GetITStatus(USART3,USART_IT_RXNE) == SET)
	{
		uint8_t Rx3Data = USART_ReceiveData(USART3);
		if(RX3State == 0)
		{
			if(Rx3Data == 0xFF)
			{
					RX3State =1;
					pRx3Packet=0;
			}
		}
		else if(RX3State == 1)
		{
				Serial_RXPacket[pRx3Packet] = Rx3Data;
				pRx3Packet++;
				if(pRx3Packet>7)
				{
					RX3State=2;
				}
		}
		else if(RX3State == 2)
		{
			if(Rx3Data == 0xFE)
			{
					RX3State = 0;
					Serial_Flag = 1;
			}
		}
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
	}
}
/********************************
函数功能 : 串口3的获取接收数据完成的标志位
输入参数 : 无
输出参数 ；1 接收完成  0 接收未完成
*********************************/
uint8_t Serial3_GetRxFlag(void)
{
	if(Serial_Flag == 1)
	{
		Serial_Flag=0;
		return 1;
	}
	return 0;
}
/********************************
函数功能 : 串口3 发送一个数组
输入参数 : 一个数组
输出参数 ；无
*********************************/
void UART3_SendArray(uint8_t *array,uint8_t length)
{
	uint8_t i;
	for(i=0;i<length;i++)
	{
		USART3_SendBits(array[i]);
	}
}
/********************************
函数功能 : 串口3 发送一个字节
输入参数 : 一个字节
输出参数 ；无
*********************************/
void USART3_SendBits(uint8_t data)
{
	USART3->DR = data;
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);
}