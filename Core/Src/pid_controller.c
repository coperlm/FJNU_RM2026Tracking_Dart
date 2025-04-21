#include "dart_control.h"
#include <math.h>

/**
 * @brief 初始化PID控制器参数
 * 
 * @param pid PID控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param output_limit 输出限幅值
 */
void pid_init(PID_t *pid, float kp, float ki, float kd, float output_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_limit = output_limit;
}

/**
 * @brief 计算PID控制器输出
 * 
 * @param pid PID控制器结构体指针
 * @param setpoint 设定值
 * @param measurement 测量值
 * @param dt 采样时间间隔(秒)
 * @return float PID控制器输出
 */
float pid_compute(PID_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    
    // 比例项
    float p_term = pid->kp * error;
    
    // 积分项 (使用积分限幅防止积分饱和)
    pid->integral += error * dt;
    float i_term = pid->ki * pid->integral;
    
    // 微分项 (使用当前误差与上次误差的差值计算)
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->kd * derivative;
    
    // 记录当前误差供下次计算使用
    pid->prev_error = error;
    
    // 计算总输出
    float output = p_term + i_term + d_term;
    
    // 对输出进行限幅
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }
    
    return output;
}
