#ifndef _OLED_H
#define _OLED_H


#include "system_init.h"
#include "stdlib.h"


#define OLED_MODE 0
#define SIZE 8
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    						  
//-----------------OLED IIC端口定义----------------  					   

#define OLED_SCLK_Clr() GPIO_ResetBits(GPIOF,GPIO_Pin_15)//SCL
#define OLED_SCLK_Set() GPIO_SetBits(GPIOF,GPIO_Pin_15)

#define OLED_SDIN_Clr() GPIO_ResetBits(GPIOF,GPIO_Pin_13)//SDA
#define OLED_SDIN_Set() GPIO_SetBits(GPIOF,GPIO_Pin_13)

 		     
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据


//OLED控制用函数
void OLED_WR_Byte(unsigned dat,unsigned cmd);  	   							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size);
void OLED_ShowString(u8 x,u8 y, u8 *p,u8 Char_Size);	 
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_IIC_Start(void);
void OLED_IIC_Stop(void);
void OLED_Write_IIC_Command(unsigned char IIC_Command);
void OLED_Write_IIC_Data(unsigned char IIC_Data);
void OLED_Write_IIC_Byte(unsigned char IIC_Byte);
void OLED_IIC_Wait_Ack(void);
void OLED_Clear(void);


void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
u32 oled_pow(u8 m,u8 n);






#endif
