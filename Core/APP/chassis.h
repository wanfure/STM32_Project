#ifndef MAILUN_CHASSIS_H
#define MAILUN_CHASSIS_H

#include <stdint.h>

// 声明外部变量：wheel_v定义在Encoder.c中
extern volatile float wheel_v;
extern float target_speed;
extern uint8_t motor_pwm;



void Chassis_Init(void);
uint8_t PID_Update(void);
void Chassis_Task(uint8_t ctrl_enable, float vx, float vy, float vw);
void Chassis_GetStatus(float* target, float* actual, uint8_t* pwm);
void kinematics_forward(float vx, float vy, float vw);

#endif //MAILUN_CHASSIS_H