#ifndef _CONTRAL_CLAW_H
#define _CONTRAL_CLAW_H

#include "system_init.h"

#define claw_most_up    215   // 195mm      64*170 = 10880脉冲                     调整上升最大距离              

#define camera_position 0   // 10mm      64*140 = 8960脉冲                        相机扫描二维码位置	
#define claw_block_get  105   // 135mm      64*153 = 9792脉冲											 物块抓取高度
#define claw_block_wait 170   //135mm                                              在物料台的物块上方等待高度
#define claw_block_put     0    // 0mm       64*153 = 9792脉冲										 放置第一层物块高度
#define claw_block_put2   105    // 90mm       64*80 = 5120脉冲                     第二层物块放置高度
#define claw_block_put3   100    //         放到第二个台阶的高度

// 放物料块的中间过程
#define get_block_down  183 // 155mm       64*127 = 8128脉冲                      转动到自己的载物台上方放置物块高度
// 靶心识别位置
#define circle_capture1  80  // 80mm       64*127 = 8128脉冲                       第一层靶心识别高度
#define circle_capture2  110  // 120mm       64*127 = 8128脉冲                     第二层靶心识别高度

typedef struct {
	int16_t position_now;             //当前值
	int16_t position_target;           //目标值
	int16_t position_temp;             //临时值（中间差值）
}CLAW_POSITION;

extern CLAW_POSITION claw;





void claw_position(int16_t position);
void claw_position2(int16_t position);
void arrive_camera(void);
void arrive_camera2(void);
void arrive_block_get(void);
void arrive_block_wait(void);
void arrive_block_put(void);
void arrive_block_put2(void);
void arrive_block_put3(void);

void arrive_most_up(void);
void arrive_block_down(void);
void arrive_circle_capture(void);
void arrive2_circle_capture(void);
void arrive_circle_capture2(void);
void arrive2_circle_capture2(void);

void claw_get_block(void);
void claw_get_block2(void);
void claw_put_block(void);
void claw_put_block2(void);
void claw_put_block3(void);


#endif
