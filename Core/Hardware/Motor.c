#include "stm32f4xx_hal.h"
#include "Motor.h"
#include "tim.h"
/**
 * @brief 电机控制函数：设置指定轮子的方向和PWM占空比
 * @param idx: 轮子索引（0=左上,1=右上,2=左下,3=右下）
 * @param dir: 方向（1=正转，-1=反转，0=停止）
 * @param pwm: PWM占空比（0~75）
 * @note 需先初始化定时器PWM输出和方向引脚为输出模式
 */
void Motor_Control(uint8_t idx, int8_t dir, uint8_t pwm) {
  // 1. PWM限幅（双重保险，防止超量程）
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  if (pwm < 0) pwm = 0;

  // 2. 停止电机：方向置0，PWM置0
  if (dir == 0) {
    pwm = 0;
  }

  // 3. 按轮子索引设置方向
  switch (idx) {
    case 0: // 左上轮
      if (dir == 1) HAL_GPIO_WritePin(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN, GPIO_PIN_SET);
      else if (dir == -1) HAL_GPIO_WritePin(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(MOTOR1_PWM_TIM, MOTOR1_PWM_CH, pwm);
      break;
    case 1: // 右上轮
      if (dir == 1) HAL_GPIO_WritePin(MOTOR2_DIR_PORT, MOTOR2_DIR_PIN, GPIO_PIN_SET);
      else if (dir == -1) HAL_GPIO_WritePin(MOTOR2_DIR_PORT, MOTOR2_DIR_PIN, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(MOTOR2_PWM_TIM, MOTOR2_PWM_CH, pwm);
      break;
    case 2: // 左下轮
      if (dir == 1) HAL_GPIO_WritePin(MOTOR3_DIR_PORT, MOTOR3_DIR_PIN, GPIO_PIN_SET);
      else if (dir == -1) HAL_GPIO_WritePin(MOTOR3_DIR_PORT, MOTOR3_DIR_PIN, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(MOTOR3_PWM_TIM, MOTOR3_PWM_CH, pwm);
      break;
    case 3: // 右下轮
      if (dir == 1) HAL_GPIO_WritePin(MOTOR4_DIR_PORT, MOTOR4_DIR_PIN, GPIO_PIN_SET);
      else if (dir == -1) HAL_GPIO_WritePin(MOTOR4_DIR_PORT, MOTOR4_DIR_PIN, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(MOTOR4_PWM_TIM, MOTOR4_PWM_CH, pwm);
      break;
    default: // 无效索引，停止所有电机
      for (int i=0; i<4; i++) {
        __HAL_TIM_SET_COMPARE(MOTOR1_PWM_TIM, TIM_CHANNEL_1+i, 0);
        HAL_GPIO_WritePin(MOTOR1_DIR_PORT, MOTOR1_DIR_PIN+i, GPIO_PIN_RESET);
      }
      break;
  }
}