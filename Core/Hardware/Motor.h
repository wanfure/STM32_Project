#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"  // 按你的芯片型号改（比如f4xx/f7xx）

// ********************* 硬件参数定义（必须改！）*********************
// 1. 电机方向引脚定义（4个轮子，每个轮子1个方向引脚）
#define MOTOR1_DIR_PIN    GPIO_PIN_0
#define MOTOR1_DIR_PORT   GPIOA
#define MOTOR2_DIR_PIN    GPIO_PIN_1
#define MOTOR2_DIR_PORT   GPIOA
#define MOTOR3_DIR_PIN    GPIO_PIN_2
#define MOTOR3_DIR_PORT   GPIOA
#define MOTOR4_DIR_PIN    GPIO_PIN_3
#define MOTOR4_DIR_PORT   GPIOA

// 2. 电机PWM定时器/通道定义（4个轮子，每个轮子1个PWM通道）
#define MOTOR1_PWM_TIM    &htim1
#define MOTOR1_PWM_CH     TIM_CHANNEL_1
#define MOTOR2_PWM_TIM    &htim1
#define MOTOR2_PWM_CH     TIM_CHANNEL_2
#define MOTOR3_PWM_TIM    &htim2
#define MOTOR3_PWM_CH     TIM_CHANNEL_1
#define MOTOR4_PWM_TIM    &htim2
#define MOTOR4_PWM_CH     TIM_CHANNEL_2

#define PWM_MAX           75          // PWM上限（和你PID限幅一致）

void Motor_Control(uint8_t idx, int8_t dir, uint8_t pwm);

#endif //MOTOR_H