#ifndef MAILUN_PID_H
#define MAILUN_PID_H




/**
 * @brief PID控制器结构体
 */
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float max_output;   // 最大输出
    float max_integral; // 积分项最大值
    float target;       // 目标值
    float actual;       // 实际值
    float error;        // 当前误差
    float last_error;   // 上次误差
    float integral;     // 积分项
    float output;       // 输出值
} PID_Controller;
/**
 * @brief 初始化PID控制器
 * @param pid: PID控制器指针
 * @param Kp: 比例系数
 * @param Ki: 积分系数
 * @param Kd: 微分系数
 * @param max_output: 最大输出
 * @param max_integral: 积分项最大值
 */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float max_output, float max_integral);
/**
 * @brief 普通PID计算函数
 * @param pid: PID控制器指针
 * @param target: 目标值
 * @param actual: 实际值
 * @return 控制输出
 */
float PID_Calculate(PID_Controller *pid, float target, float actual);

#endif //MAILUN_PID_H