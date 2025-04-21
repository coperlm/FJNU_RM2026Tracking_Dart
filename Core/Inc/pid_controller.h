#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

// 扩展PID结构体定义
typedef struct {
    // 基本PID参数
    float kp;                    // 比例系数
    float ki;                    // 积分系数
    float kd;                    // 微分系数
    float kff;                   // 前馈系数 (新增)
    
    // 状态变量
    float integral;              // 积分项
    float prev_error;            // 上一次误差
    float prev_derivative;       // 上一次微分值 (新增)
    float output_limit;          // 输出限幅
    float integral_limit;        // 积分限幅 (新增)
    
    // 高级控制特性
    float d_lpf_beta;            // D项低通滤波系数 (新增)
    bool anti_windup;            // 是否使用积分反饱和 (新增)
    bool d_term_lpf;             // 是否使用D项低通滤波 (新增)
    
    // 诊断数据
    float last_output;           // 上一次输出 (新增，用于诊断)
    uint32_t last_calc_time;     // 上一次计算时间 (新增，用于自适应dt)
} PID_t;

// PID控制器初始化和更新函数
void pid_init(PID_t *pid, float kp, float ki, float kd, float output_limit);
float pid_compute(PID_t *pid, float setpoint, float measurement, float dt);

// 高级PID控制器初始化和更新函数 (新增)
void pid_init_advanced(PID_t *pid, float kp, float ki, float kd, float kff, float output_limit, float integral_limit);
float pid_compute_advanced(PID_t *pid, float setpoint, float measurement, float feedforward, float dt);

// 设置PID高级参数 (新增)
void pid_set_anti_windup(PID_t *pid, bool enable);
void pid_set_d_lpf(PID_t *pid, bool enable, float beta);
void pid_reset(PID_t *pid);

#endif /* PID_CONTROLLER_H */