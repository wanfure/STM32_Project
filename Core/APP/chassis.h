#ifndef MAILUN_CHASSIS_H
#define MAILUN_CHASSIS_H

#include <stdint.h>
#include "PID.h"


// 声明外部变量：wheel_v定义在Encoder.c中
extern volatile float wheel_v;
extern float target_speed;
extern uint8_t motor_pwm;

/* ==================== 物理参数（仅用于速度缩放）==================== */
#define WHEEL_RADIUS    0.03f    // 轮子半径(m)
#define WHEEL_BASE      0.14f    // 轴距a+b (m)
#define MAX_WHEEL_RPS   50.0f    // 电机最大转速(r/s)（实测值，用于限幅）
#define CODE_MAX_SPEED       200.0f


 //* @brief  电机最大转速(r/s)（实测值，用于限幅）
#define MOTOR_MAX_RPS     308.33f  // 18500rpm ÷ 60 ≈ 308.33转/秒


/* ==================== 全局变量 ==================== */
static PID_Controller speed_pid[4];  // 4个轮子独立PID
float wheel_target_rps[4];           // 轮子目标转速(r/s)（带方向）
float wheel_actual_rps[4];           // 轮子实际转速(r/s)
int8_t wheel_dir[4];                 // 轮子方向（1正转/-1反转/0停止）
uint8_t wheel_pwm[4];                // 轮子最终PWM值（0~75）



void Chassis_Init(void);
uint8_t PID_Update(uint8_t idx, float target, float actual);
void kinematics_forward(float vx, float vy, float vw, float wheel[4]);
float Speed2PWM(float raw_speed, int8_t *dir);
void Chassis_Task(uint8_t ctrl_enable, float vx, float vy, float vw);

#endif //MAILUN_CHASSIS_H