#include "chassis.h"
#include <stdint.h>
#include "Encoder.h"
#include "Motor.h"
#include "math.h"


/* ==================== PID参数配置 ==================== */
#define SPEED_KP        8.0f
#define SPEED_KI        2.0f
#define SPEED_KD        0.5f
#define MAX_PWM         100.0f  // PWM最大输出值（对应100%占空比）
#define MAX_INTEGRAL    50.0f   // 积分限幅，防止积分饱和




/**
* @brief 底盘初始化
* 初始化PID控制器，设置Kp、Ki、Kd等参数
*/
void Chassis_Init(void)
{
    for (int i = 0; i < 4; i++) {
        PID_Init(&speed_pid[i], SPEED_KP, SPEED_KI, SPEED_KD, MAX_PWM, MAX_INTEGRAL);
    }
}

uint8_t PID_Update(uint8_t idx, float target, float actual) {
    float pid_out = PID_Calculate(&speed_pid[idx], target, actual);
    // PWM限幅，确保0~75
    pid_out = pid_out > MAX_PWM ? MAX_PWM : (pid_out < 0 ? 0 : pid_out);
    return (uint8_t)pid_out;
}


/**
 * @brief 麦克纳姆轮底盘运动学正解
 * @功能 由底盘整体速度（摇杆输入）计算四个轮子的目标速度，保证正负（正反转）不变，且速度不超限
 * @参数 vx: 底盘前后方向速度（摇杆值，范围-100~100）
 * @参数 vy: 底盘左右方向速度（摇杆值，范围-100~100）
 * @参数 vw: 底盘自转速度（摇杆值，范围-100~100）
 * @参数 wheel[4]: 输出参数，存放四个轮子的最终速度，顺序为：
 *        wheel[0] = 左上轮（左前轮）、wheel[1] = 右上轮（右前轮）
 *        wheel[2] = 左下轮（左后轮）、wheel[3] = 右下轮（右后轮）
 * @说明 1. 轮子速度正负决定正反转，正数/负数对应电机正转/反转
 *       2. 所有轮子等比例缩放，保证底盘运动方向不偏移
 */
void kinematics_forward(float vx, float vy, float vw, float wheel[4]) {


    // 第一步：基础速度计算（核心公式，正负保留，决定轮子正反转）
    wheel[0] = vx + vy + vw;  // 左上轮速度公式
    wheel[1] = vx - vy - vw;  // 右上轮速度公式
    wheel[2] = vx - vy + vw;  // 左下轮速度公式
    wheel[3] = vx + vy - vw;  // 右下轮速度公式

    // 第二步：计算四个轮子速度的最大绝对值（仅关注速度大小，不关注方向）
    float max_abs_speed = 0.0f;  // 初始化最大绝对值为0
    for (int i = 0; i < 4; i++) {
        // 取当前轮子速度的绝对值，和已记录的最大值比较，保留更大的那个
        max_abs_speed = fmax(max_abs_speed, fabs(wheel[i]));
    }

    // 第三步：等比例缩放（仅当最大速度超过阈值时执行，保证方向不偏）
    if (max_abs_speed > CODE_MAX_SPEED) {
        // 计算缩放系数（0~1之间，保证所有轮子速度等比例缩小）
        float scale = CODE_MAX_SPEED / max_abs_speed;
        // 所有轮子按同一系数缩放，正负号保留（正反转不变）
        for (int i = 0; i < 4; i++) {
            wheel[i] *= scale;
        }
    }
    // 若最大速度未超限，直接使用原始计算值，无需缩放
}

/**
 * @brief 分离速度和方向（摇杆值正负直接对应电机转向）
 * @param speed: 轮子目标速度（带方向，来自摇杆解算）
 * @param idx: 轮子索引
 */
void Split_Speed_Dir(float speed, uint8_t idx) {
    wheel_dir[idx] = 0;
    if (speed > 0.1f) {          // 正转阈值（防抖）
        wheel_dir[idx] = 1;
        wheel_target_rps[idx] = speed * (MOTOR_MAX_RPS / CODE_MAX_SPEED); // 目标转速取正值
    } else if (speed < -0.1f) {  // 反转阈值（防抖）
        wheel_dir[idx] = -1;
        wheel_target_rps[idx] = fabs(speed) * (MOTOR_MAX_RPS / CODE_MAX_SPEED); // 目标转速取绝对值
    } else {
        wheel_target_rps[idx] = 0; // 停止
    }
}



void Chassis_Task(uint8_t en, float vx, float vy, float vw){
    if(en == 0){
        for(int i=0; i<4; i++) Motor_Control(i, 0, 0);
        return;
    }

    //运动学正解：摇杆值直接计算4个轮子的目标速度（带方向）
    kinematics_forward(vx, vy, vw, wheel_target_rps);

    //逐轮闭环控制（核心：摇杆值→方向+PWM+闭环）
    for (int i = 0; i < 4; i++) {
        // 3.1 分离方向和目标转速
        Split_Speed_Dir(wheel_target_rps[i], i);
        if (wheel_dir[i] == 0) { // 停止则跳过
            Motor_Control(i, 0, 0);
            continue;
        }

        // 3.2 读取当前轮子的实际转速（编码器反馈，闭环关键）
        wheel_actual_rps[i] = rotate_v_s;

        // 3.3 PID闭环计算：目标转速 vs 实际转速
        wheel_pwm[i] = PID_Update(i, wheel_target_rps[i], wheel_actual_rps[i]);

        // 3.4 控制电机：摇杆值决定的方向 + 闭环后的PWM
        Motor_Control(i, wheel_dir[i], wheel_pwm[i]);
    }

}