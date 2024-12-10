#ifndef _UART_H
#define _UART_H

#include "sys.h"
#include <stdio.h>

extern uint8_t Serial_TXPacket[3];
extern uint8_t Serial_RXPacket[8];
extern uint8_t Serial2_TXPacket[3];
extern uint8_t Serial2_RXPacket[8];
extern uint8_t Serial3_TXPacket[3];
extern uint8_t Serial3_RXPacket[11];
extern uint8_t Serial3_HW101_Angle[2];

extern uint8_t Serial4_TXPacket[];
extern uint8_t Serial4_RXPacket[];
extern uint8_t Serial4_HW101_Angle[];

extern uint8_t USART2_TX_BUF[100];
extern uint8_t UART4_RX_BUF[100];

extern volatile float global_angle;
extern volatile uint8_t new_data_received;
extern volatile float angular_velocity_y;
extern volatile float angular_velocity_z;

/********************串口1************************/
void UART1_Init(void);
void USART1_SendBits(uint8_t data);
void UART1_SendArray(uint8_t *array,uint8_t length);
void UART1_SendString(uint8_t *String1);
void UART3_SendNum(uint32_t Number,uint8_t Length);
uint32_t Serial_Pow(uint32_t x,uint32_t y);


void UART_SendPacket(uint8_t *Serial_TXPacket);
uint8_t Serial1_GetRxFlag(void);
void USART1_IRQHandler(void);
void UART_SendPacket2UP(uint8_t data);
void UART_SendPacket3UP(uint8_t data);
/********************串口2************************/
void UART2_Init(void);
void USART2_SendBits(uint8_t data);
int fputc(int ch, FILE *f);
void USART2_IRQHandler(void);

void u2_printf(char* fmt,...);


/********************串口3************************/
void UART3_Init(void);
void USART3_IRQHandler(void);
void UART3_SendArray(uint8_t *array,uint8_t length);
void USART3_SendBits(uint8_t data);

/********************串口4************************/
void UART4_Init(void);
void UART4_IRQHandler(void);
void UART4_SendArray(uint8_t *array,uint8_t length);
void UART4_SendBits(uint8_t data);
void Serial4_SendPacket(void);
uint8_t Serial4_GetRxFlag (void);
void ParseData(uint8_t *data, uint16_t length);
uint8_t CalculateChecksum(uint8_t *data, uint16_t length, uint8_t type) ;
	
#endif
