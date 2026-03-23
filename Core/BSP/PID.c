#include "PID.h"



/**
 * @brief 初始化PID控制器
 * @param pid: PID控制器指针
 * @param Kp: 比例系数
 * @param Ki: 积分系数
 * @param Kd: 微分系数
 * @param max_output: 最大输出
 * @param max_integral: 积分项最大值
 */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float max_output, float max_integral) {
    pid->Kp           = Kp;         // 设置比例系数
    pid->Ki           = Ki;         // 设置积分系数
    pid->Kd           = Kd;         // 设置微分系数
    pid->max_output   = max_output; // 设置最大输出
    pid->max_integral = max_integral; // 设置积分项最大值
    pid->target       = 0.0f;       // 初始化目标值为0
    pid->actual       = 0.0f;       // 初始化实际值为0
    pid->error        = 0.0f;       // 初始化误差为0
    pid->last_error   = 0.0f;       // 初始化上次误差为0
    pid->integral     = 0.0f;       // 初始化积分为0
    pid->output       = 0.0f;       // 初始化输出为0
}

/**
 * @brief 普通PID计算函数
 * @param pid: PID控制器指针
 * @param target: 目标值
 * @param actual: 实际值
 * @return 控制输出
 */
float PID_Calculate(PID_Controller *pid, float target, float actual) {
    pid->target = target;  // 更新目标值
    pid->actual = actual;  // 更新实际值
    pid->error = pid->target - pid->actual;  // 计算当前误差

    float P = pid->Kp * pid->error;  // 比例项计算

    pid->integral += pid->error;  // 积分项累加
    // 积分饱和处理
    if (pid->integral > pid->max_integral)
        pid->integral = pid->max_integral;
    else if (pid->integral < -pid->max_integral)
        pid->integral = -pid->max_integral;
    float I = pid->Ki * pid->integral;  // 积分项计算

    float D = pid->Kd * (pid->error - pid->last_error);  // 微分项计算
    pid->last_error = pid->error;  // 保存当前误差作为下次的上次误差

    pid->output = P + I + D;  // 计算总输出
    // 输出限幅
    if (pid->output > pid->max_output)
        pid->output = pid->max_output;
    else if (pid->output < -pid->max_output)
        pid->output = -pid->max_output;

    return pid->output;  // 返回计算得到的输出
}


