#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "dart_control.h" // 包含dart_control.h以使用其中的PID_t定义

// 扩展PID控制器结构体，为先前定义的PID_t添加高级功能
typedef struct {
    PID_t base_pid;          // 基本PID结构体
    float kff;               // 前馈增益
    bool anti_windup;        // 积分饱和限制
    float integral_limit;    // 积分项限制
    bool d_term_lpf;         // 微分项低通滤波器
    float d_lpf_beta;        // 微分低通滤波系数
    float prev_measurement;  // 上一次的测量值
} PID_Advanced_t;

// 高级PID控制器初始化和更新函数
void pid_init_advanced(PID_Advanced_t *pid, float kp, float ki, float kd, float kff, float output_limit, float integral_limit);
float pid_compute_advanced(PID_Advanced_t *pid, float setpoint, float measurement, float feedforward, float dt);

// 设置PID高级参数
void pid_set_anti_windup(PID_Advanced_t *pid, bool enable);
void pid_set_d_lpf(PID_Advanced_t *pid, bool enable, float beta);
void pid_reset_advanced(PID_Advanced_t *pid);

// 确保与原始PID函数声明兼容
// 这里不重复声明基本PID函数，使用dart_control.h中的声明

#endif /* PID_CONTROLLER_H */
