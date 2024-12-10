#include "contral_claw.h"

CLAW_POSITION claw;
/********************
函数功能 : 爪子位置函数  单位mm
输入参数 : 无
输出参数 ：无
直径 16mm 
转一圈  8*2*3.14=50.24 
x/mm = 3200/50.24
x = 64*mm
**********************/
void claw_position(int16_t position)
{
	uint32_t pulse=0;
	claw.position_now = claw.position_now + position; // 更新当前位置
	
	if(claw.position_now<0 || claw.position_now>220)
	{
		claw.position_now=claw.position_now - position;
	}
	else
	{
		// 方向
		if(position>0)
		{
				Motor1_DIR(1);   // 向上
		}
		else
		{
				position=-position;
				Motor1_DIR(0);   // 向下
		}
		pulse=(64*position);
		distance1=pulse;
		stepPosition1=0;
		MSD_Move1(pulse,32,32,64);
		while(1)
		{
			if(stepPosition1 == distance1)
			{
				break;
			}
		}
	}
}
/********************
函数功能 : 爪子位置函数  单位mm
输入参数 : 无
输出参数 ：无
是否结束由外部控制
*********/
void claw_position2(int16_t position)
{
	uint32_t pulse=0;
	claw.position_now = claw.position_now + position; // 更新当前位置
	if(claw.position_now<0 || claw.position_now>220)
	{
		claw.position_now=claw.position_now - position;
	}
	else
	{
		// 方向
		if(position>0)
		{
				Motor1_DIR(1);   // 向上
		}
		else
		{
				position=-position;
				Motor1_DIR(0);   // 向下
		}
		pulse=(64*position);
		distance1=pulse;
		stepPosition1=0;
		MSD_Move1(pulse,32,32,64);//原来是4 4 8
	}
}


/********************
函数功能 : 到达摄像头的位置
输入参数 : 无
输出参数 ：无
**********************/
void arrive_camera(void)
{
	claw.position_target = camera_position;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}
/********************
函数功能 : 到达摄像头的位置
输入参数 : 无
输出参数 ：无
是否结束由外部控制
**********************/
void arrive_camera2(void)
{
	claw.position_target = camera_position;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position2(claw.position_temp);
}
/********************
函数功能 : 到达抓物料块的位置
输入参数 : 无
输出参数 ：无
**********************/
void arrive_block_get(void)
{
	claw.position_target = claw_block_get;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}
/********************
函数功能 : 到达物料台上方等待
输入参数 : 无
输出参数 ：无
**********************/
void arrive_block_wait(void)
{
	claw.position_target = claw_block_wait;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}
/********************
函数功能 : 到达放物料块的位置
输入参数 : 无
输出参数 ：无
**********************/
void arrive_block_put(void)
{
	claw.position_target = claw_block_put;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}
/********************
函数功能 : 到达放物料块的位置  放第二层
输入参数 : 无
输出参数 ：无
**********************/
void arrive_block_put2(void)
{
	claw.position_target = claw_block_put2;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}
/********************
函数功能 : 到达放物料块的位置  放第二层
输入参数 : 无
输出参数 ：无
**********************/
void arrive_block_put3(void)
{
	claw.position_target = claw_block_put3;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}
/********************
函数功能 : 到达最顶端
输入参数 : 无
输出参数 ：无
**********************/
void arrive_most_up(void)
{
	claw.position_target = claw_most_up;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);
}
/********************
函数功能 : 从转盘拿物料块放到载物台下降的位置
输入参数 : 无
输出参数 ：无
**********************/
void arrive_block_down(void)
{
	claw.position_target = get_block_down;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);	
}
/********************
函数功能 : 到靶心识别的位置
输入参数 : 无
输出参数 ：无
**********************/
void arrive_circle_capture(void)
{
	claw.position_target = circle_capture1;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);	
}
void arrive2_circle_capture(void)
{
	claw.position_target = circle_capture1;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position2(claw.position_temp);	
}
/********************
函数功能 : 到靶心识别的位置2
输入参数 : 无
输出参数 ：无
**********************/
void arrive_circle_capture2(void)
{
	claw.position_target = circle_capture2;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position(claw.position_temp);	
}
void arrive2_circle_capture2(void)
{
	claw.position_target = circle_capture2;
	claw.position_temp = claw.position_target - claw.position_now;
	claw_position2(claw.position_temp);	
}
/********************
函数功能 : 爪子把物块放到载物台   从转盘抓
输入参数 : 无
输出参数 ：无
**********************/
void claw_get_block(void)
{
	arrive_block_get();  //160
	delay_ms(200);
	claw_close(); 
	delay_ms(500);
	arrive_most_up();    //195 
	claw_turn129(); 
	delay_ms(200);
	delay_ms(500); 
	arrive_block_down(); //130 
	claw_open();
	delay_ms(500); 
	arrive_most_up();    //173 
	claw_turn0();
	delay_ms(200);
	delay_ms(500);
	support_turn120();
	delay_ms(100);
}
/********************
函数功能 : 爪子把物块放到载物台   从地面抓
输入参数 : 无
输出参数 ：无
**********************/
void claw_get_block2(void)
{
	arrive_block_put();   //0
	claw_close();
	arrive_most_up();     //173
	delay_ms(200);
	claw_turn129();
	delay_ms(200);
	arrive_block_down();  //130
	claw_open();
	arrive_most_up();     //173
	claw_turn0();
	delay_ms(200);
	support_turn120();	
	delay_ms(100);
	
}
/********************
函数功能 : 爪子吧物块拿出载物台 放到第一层
输入参数 : 无
输出参数 ：无
**********************/
void claw_put_block(void)
{
	claw_open();
	arrive_most_up();
	delay_ms(500);	//爪子打物块，升最高停一下
	claw_turn129();
	delay_ms(1500);
	arrive_block_down();
	claw_close();
	delay_ms(300);	
	arrive_most_up(); 
	delay_ms(500);
	claw_turn0();
	delay_ms(300);
	support_turn120();
	arrive_block_put(); 
	delay_ms(300);	
	claw_open();
	delay_ms(300);
}
/********************
函数功能 : 爪子吧物块拿出载物台 放到第二层
输入参数 : 无
输出参数 ：无
**********************/
void claw_put_block2(void)
{
	claw_open();
	arrive_most_up();       
	claw_turn129();
	delay_ms(200);
	arrive_block_down();
	claw_close();
	delay_ms(300);
	arrive_most_up();   
	claw_turn0();
	delay_ms(300);
	support_turn120();
	arrive_block_put2(); 
	delay_ms(300);	
	claw_open();
	delay_ms(300);
}
/********************
函数功能 : 爪子吧物块拿出载物台 放到第二层平台上
输入参数 : 无
输出参数 ：无
**********************/
void claw_put_block3(void)
{
	claw_open();
	arrive_most_up();       
	claw_turn129();
	delay_ms(200);
	arrive_block_down();
	claw_close();
	delay_ms(300);
	arrive_most_up();   
	claw_turn0();
	delay_ms(300);
	support_turn120();
	arrive_block_put3(); 
	delay_ms(300);	
	claw_open();
	delay_ms(300);
}
