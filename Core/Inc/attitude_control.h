#ifndef ATTITUDE_CONTROL_H
#define ATTITUDE_CONTROL_H

#include "dart_control.h"

// 控制参数
#define ROLL_KP  2.0f
#define ROLL_KI  0.05f
#define ROLL_KD  0.5f
#define ROLL_MAX 45.0f  // 最大横滚角度修正

#define PITCH_KP  2.0f
#define PITCH_KI  0.05f
#define PITCH_KD  0.5f
#define PITCH_MAX 45.0f  // 最大俯仰角度修正

#define YAW_KP  1.5f
#define YAW_KI  0.02f
#define YAW_KD  0.3f
#define YAW_MAX 90.0f  // 最大偏航角度修正

#define TARGET_KP  0.8f
#define TARGET_KI  0.01f
#define TARGET_KD  0.2f
#define TARGET_MAX 1.0f  // 目标跟踪最大修正值

// PWM参数映射
#define SERVO_CENTER       1500    // 舵机中位值(微秒)
#define SERVO_TRAVEL       500     // 舵机行程(微秒)
#define MOTOR_IDLE         1100    // 电机怠速值(微秒)
#define MOTOR_MAX_THRUST   1800    // 电机最大推力值(微秒)

/**
 * @brief 初始化飞镖系统
 */
void dart_system_init(void);

/**
 * @brief 根据目标位置计算舵机和电机控制量
 * 
 * @param attitude 陀螺仪姿态数据
 * @param target 目标相对位置数据
 * @param control 输出的控制值
 */
void target_tracking(const Attitude_t *attitude, const Target_t *target, Control_t *control);

/**
 * @brief 更新姿态控制输出
 * 
 * @param attitude 陀螺仪姿态数据
 * @param target 目标相对位置数据
 * @param control 输出的控制值
 */
void update_attitude_control(const Attitude_t *attitude, const Target_t *target, Control_t *control);

#endif /* ATTITUDE_CONTROL_H */