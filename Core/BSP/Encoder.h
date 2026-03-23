#ifndef MAILUN_ENCODER_H
#define MAILUN_ENCODER_H

#include "main.h"

#define ENCODER_PPR 1024  // 单脉冲编码器1024脉冲/圈
#define COUNT_PER_SEC 10  // 100ms算一次转速（1秒10次）
#define  WHEEL     30     //轮半径30mm


extern volatile float rotate_v_s ;

void Encoder_Init(void);
int16_t Encoder_Read(void);
float Encoder_GetSpeed(void);
void UART_Printf(const char *format,...) ;
uint8_t GPIO_Check_Level(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void re_direction(void) ;
void Encoder_Report() ;

#endif //MAILUN_ENCODER_H