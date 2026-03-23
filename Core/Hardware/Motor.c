#include "stm32f4xx_hal.h"
#include "Motor.h"
#include "tim.h"

void Motor_Init(void)
{
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  Mecanum_Wheel_Control(1, 0, 1);
  Mecanum_Wheel_Control(2, 0, 1);
  Mecanum_Wheel_Control(3, 0, 1);
  Mecanum_Wheel_Control(4, 0, 1);

}

// wheel: 1=左上 2=右上 3=左下 4=右下
// duty_cycle: 0-100（占空比，速度）
// dir: 1=正转 0=反转（麦轮需独立控方向）
void Mecanum_Wheel_Control(uint8_t wheel, uint8_t duty_cycle, uint8_t dir)
{
  switch(wheel)
  {
    case 1: // 左上轮：A0(PWM)、C0(正)、C15(反)
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
      break;

    case 2: // 右上轮：A1(PWM)、C1(正)、C2(反)
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty_cycle);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
      break;

    case 3: // 左下轮：A2(PWM)、C13(正)、C14(反)
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty_cycle);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
      break;

    case 4: // 右下轮：A3(PWM)、C4(正)、C5(反)
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty_cycle);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
      break;
  }
}

// speed: 0-100（整体速度）
void Mecanum_Move_Forward(uint8_t speed)  { /* 前进 */
  Mecanum_Wheel_Control(1, speed, 1);
  Mecanum_Wheel_Control(2, speed, 0);
  Mecanum_Wheel_Control(3, speed, 1);
  Mecanum_Wheel_Control(4, speed, 0);
}
void Mecanum_Move_Backward(uint8_t speed) { /* 后退 */
  Mecanum_Wheel_Control(1, speed, 0);
  Mecanum_Wheel_Control(2, speed, 1);
  Mecanum_Wheel_Control(3, speed, 0);
  Mecanum_Wheel_Control(4, speed, 1);
}
void Mecanum_Move_Left(uint8_t speed)     { /* 左平移 */
  Mecanum_Wheel_Control(1, speed, 0);
  Mecanum_Wheel_Control(2, speed, 0);
  Mecanum_Wheel_Control(3, speed, 1);
  Mecanum_Wheel_Control(4, speed, 1);
}
void Mecanum_Move_Right(uint8_t speed)    { /* 右平移 */
  Mecanum_Wheel_Control(1, speed, 1);
  Mecanum_Wheel_Control(2, speed, 1);
  Mecanum_Wheel_Control(3, speed, 0);
  Mecanum_Wheel_Control(4, speed, 0);
}
void Mecanum_Rotate_Left(uint8_t speed)    { /* 原地左转 */
  Mecanum_Wheel_Control(1, speed, 0);
  Mecanum_Wheel_Control(2, speed, 0);
  Mecanum_Wheel_Control(3, speed, 0);
  Mecanum_Wheel_Control(4, speed, 0);
}
void Mecanum_Rotate_Right(uint8_t speed)    { /* 原地右转 */
  Mecanum_Wheel_Control(1, speed, 1);
  Mecanum_Wheel_Control(2, speed, 1);
  Mecanum_Wheel_Control(3, speed, 1);
  Mecanum_Wheel_Control(4, speed, 1);
}
void Mecanum_Stop(void)                   { /* 停止 */
  Mecanum_Wheel_Control(1, 0, 1);
  Mecanum_Wheel_Control(2, 0, 1);
  Mecanum_Wheel_Control(3, 0, 1);
  Mecanum_Wheel_Control(4, 0, 1);
}