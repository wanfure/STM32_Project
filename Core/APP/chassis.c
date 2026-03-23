#include <stdint.h>
#include "chassis.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "math.h"


/* ==================== PID参数配置 ==================== */
#define SPEED_KP        8.0f
#define SPEED_KI        2.0f
#define SPEED_KD        0.5f
#define MAX_PWM         100.0f  // PWM最大输出值（对应100%占空比）
#define MAX_INTEGRAL    50.0f   // 积分限幅，防止积分饱和


/* ==================== 全局变量 ==================== */
static PID_Controller speed_pid;        // PID控制器实例
float target_speed = 0.0f;   // 当前目标速度（带正负方向）
static uint8_t current_pwm = 0;         // 当前实际输出的PWM值

uint8_t  motor_pwm;
int8_t pwm;

/////
#define S       0.03f   // 轮子半径m
#define A_PLUS_B 0.14f  // a+b 的值（轮子到旋转中心的距离，单位：m


/**
* @brief 底盘初始化
* 初始化PID控制器，设置Kp、Ki、Kd等参数
*/
void Chassis_Init(void)
{
   PID_Init(&speed_pid, SPEED_KP, SPEED_KI, SPEED_KD, MAX_PWM, MAX_INTEGRAL);
}

uint8_t PID_Update(void){
   // PID计算：输入目标速度和实际速度，返回调整量
   // wheel_v来自Encoder.c，是编码器测出的实际电机转速(r/s)
   float pid_out = PID_Calculate(&speed_pid, motor_pwm, rotate_v_s);

   // 基础PWM（根据目标速度开环计算）+ PID调整量
    pwm = pid_out;

   // PWM限幅，确保在0-100之间
   if (pwm > 75) pwm = 75;
   if (pwm < 0) pwm = 0;

    return (uint8_t)pwm;
}


void Chassis_Task(uint8_t ctrl_enable, float vx, float vy, float vw){

    kinematics_forward( vx, vy, vw);

    // 控制禁用时直接停止
    if (ctrl_enable == 0) {
        Mecanum_Stop();
        return;
    }

    // 前进/后退 > 左移/右移 > 旋转
    if (fabs(vx) > 0.1) {
        // 前进/后退
        if (vx > 0) {
            Mecanum_Move_Forward(pwm);
        } else {
            Mecanum_Move_Backward(pwm);
        }
    } else if (fabs(vy) > 0.1) {
        // 左移/右移
        if (vy < 0) {
            Mecanum_Move_Left(pwm);
        } else {
            Mecanum_Move_Right(pwm);
        }
    } else if (fabs(vw) > 0.1) {
        // 旋转
        if (vw < 0) {
            Mecanum_Rotate_Left(pwm);
        } else {
            Mecanum_Rotate_Right(pwm);
        }
    } else {
        // 停止
        Mecanum_Stop();
    }

}

/**
* @brief 获取当前底盘状态（用于调试）
* @param target: 输出当前目标速度(m/s)
* @param actual: 输出编码器测得的实际速度(m/s)
* @param pwm: 输出当前PWM值
*/
void Chassis_GetStatus(float* target, float* actual, uint8_t* pwm)
{
   *target = target_speed;   // 当前设定的目标速度
   *actual = wheel_v;            // 来自Encoder.c的实际速度
   *pwm = current_pwm;           // 当前输出的PWM值
}


/**
 * @brief 运动学正解：由底盘速度计算四个轮子的角速度
 * @param vx 摇杆值（-100到100）
 * @param vy 摇杆值（-100到100）
 * @param vw 杆值（-100到100）
 * @param omega_out 输出四个轮子的角速度 [ω0, ω1, ω2, ω3]
 * @param motor_out 输出四个电机的转
 *
 */




void kinematics_forward(float vx, float vy, float vw) {
    // 公式：(1/s) * 矩阵 * [vx, vy, vw]^T = [ω0, ω1, ω2, ω3]^T


    float motor_out[4];
    float omega_out[4];
    omega_out[0] = (-1 * vx + 1 * vy + A_PLUS_B * vw) / S;
    omega_out[1] = (-1 * vx - 1 * vy + A_PLUS_B * vw) / S;
    omega_out[2] = (1 * vx - 1 * vy + A_PLUS_B * vw) / S;
    omega_out[3] = (1 * vx + 1 * vy + A_PLUS_B * vw) / S;

    motor_out[0] = omega_out[0] /2 / 3.1415926f*5;
    motor_out[1] = omega_out[1] /2 / 3.1415926f*5;
    motor_out[2] = omega_out[2] /2 / 3.1415926f*5;
    motor_out[3] = omega_out[3] /2 / 3.1415926f*5;


      motor_pwm=motor_out[0]/308*100;
}

