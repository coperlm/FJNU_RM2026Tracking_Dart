#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "dart_control.h"

/**
 * @brief 初始化PID控制器参数
 * 
 * @param pid PID控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param output_limit 输出限幅值
 */
void pid_init(PID_t *pid, float kp, float ki, float kd, float output_limit);

/**
 * @brief 计算PID控制器输出
 * 
 * @param pid PID控制器结构体指针
 * @param setpoint 设定值
 * @param measurement 测量值
 * @param dt 采样时间间隔(秒)
 * @return float PID控制器输出
 */
float pid_compute(PID_t *pid, float setpoint, float measurement, float dt);

#endif /* PID_CONTROLLER_H */