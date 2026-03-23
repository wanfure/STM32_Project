
#ifndef MOTOR_H
#define MOTOR_H

void Motor_Init(void);
void Mecanum_Wheel_Control(uint8_t wheel, uint8_t duty_cycle, uint8_t dir);
void Mecanum_Move_Forward(uint8_t speed);
void Mecanum_Move_Backward(uint8_t speed);
void Mecanum_Move_Right(uint8_t speed);
void Mecanum_Move_Left(uint8_t speed);
void Mecanum_Rotate_Right(uint8_t speed);
void Mecanum_Rotate_Left(uint8_t speed);
void Mecanum_Stop(void);

#endif //MOTOR_H