#include "Encoder.h"
#include "usart.h"
#include "tim.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "chassis.h"
#include "elrs.h"

uint8_t level1;

volatile uint32_t pulse_cnt = 0;  // 脉冲总数（中断累加）
volatile float rpm = 0.0f;        // 最终转速
volatile float rotate_v_s = 0.0f;        // 每秒转速小齿轮电机
volatile float rotate_v_m = 0.0f;        // 每秒转速中齿轮
volatile float rotate_v_l= 0.0f;        // 每秒转速大齿轮
volatile float Omega=0.0f;// 转向角速度



volatile float wheel_v = 0.0f;        // 轮速M/s


void UART_Printf(const char *format,...) {
    char tmp[50];

    va_list argptr;
    va_start(argptr, format);
    vsnprintf(tmp, sizeof(tmp), format, argptr); // 用vsnprintf，避免缓冲区溢出
    // vsprintf((char*)tmp,format,argptr);
    va_end(argptr);
    HAL_UART_Transmit_IT(&huart4, (const uint8_t *)&tmp, strlen(tmp));
}
uint8_t GPIO_Check_Level(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    // 直接读取引脚电平，返回直观的1/0
    return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET) ? 1 : 0;
}


//方向判断
void re_direction(void) {
    level1 = GPIO_Check_Level(GPIOA, GPIO_PIN_8);
    if(level1 == 1)
    {
        // 高电平逻辑
        UART_Printf("direction:%d\r\n",level1);
    }
    else
    {
        // 低电平逻辑
        UART_Printf("direction:%d\r\n",level1);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    // TIM1通道1的中断
    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        // 1. 彻底清除中断标志
        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);

        // 2. 硬件防抖：
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET) {
            pulse_cnt++;  // 脉冲时计数
        }

        // 3. 重启捕获
        HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        // 算转速：rpm = (脉冲数/秒) * 60 / 每圈脉冲数
       // rpm = (float)pulse_cnt * COUNT_PER_SEC * 60 / ENCODER_PPR;
        // 每秒转速 = (100ms脉冲数) × (1秒的100ms个数) ÷ 每圈脉冲数
        rotate_v_m = (float)pulse_cnt * COUNT_PER_SEC / ENCODER_PPR;
       // angular_v_min = cps * 6.28319f;
        rotate_v_l = rotate_v_m * 0.42857f;//大齿轮70，中齿轮30 齿

        wheel_v=rotate_v_l*2*3.14159265358*WHEEL/1000;//米每秒

        rotate_v_s=rotate_v_l*5;//电机齿轮14

        Omega=wheel_v*0.707106/0.07179311;//转向角速度

        pulse_cnt = 0;  // 清零，准备下一个100ms
    }

}

// 报告编码器数据
void Encoder_Report() {
    // re_direction();//方向1是顺时针
// UART_Printf("RPM:%.1f\r\n", rpm);
//HAL_Delay(100);
//UART_Printf("CPS:%.1f\r\n",cps);//每秒转速

//UART_Printf("angular_v_max:%.1f\r\n",angular_v_max);//每秒角速度
  //  UART_Printf("speed:%.1fm/s\r\n",wheel_v);//米每秒
//HAL_Delay(100);
    UART_Printf("vx:%d vy:%d vw:%d\r\n",ch3_map,ch4_map,ch1_map);
}
