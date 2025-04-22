#include "pid_controller.h"
#include "main.h"
#include <math.h>

/**
 * @brief 初始化基本PID控制器参数
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
 * @brief 计算基本PID控制器输出
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
    
    // 积分项
    pid->integral += error * dt;
    float i_term = pid->ki * pid->integral;
    
    // 微分项
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - pid->prev_error) / dt;
    }
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

/**
 * @brief 初始化高级PID控制器参数
 * 
 * @param pid 高级PID控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param kff 前馈系数
 * @param output_limit 输出限幅值
 * @param integral_limit 积分限幅值
 */
void pid_init_advanced(PID_Advanced_t *pid, float kp, float ki, float kd, float kff, float output_limit, float integral_limit) {
    // 初始化基本PID参数
    pid->base_pid.kp = kp;
    pid->base_pid.ki = ki;
    pid->base_pid.kd = kd;
    pid->base_pid.integral = 0.0f;
    pid->base_pid.prev_error = 0.0f;
    pid->base_pid.output_limit = output_limit;
    
    // 初始化高级PID参数
    pid->kff = kff;
    pid->integral_limit = integral_limit;
    pid->anti_windup = true;
    pid->d_term_lpf = true;
    pid->d_lpf_beta = 0.7f;
    pid->prev_measurement = 0.0f;
}

/**
 * @brief 设置PID积分反饱和功能
 * 
 * @param pid 高级PID控制器结构体指针
 * @param enable 是否启用积分反饱和
 */
void pid_set_anti_windup(PID_Advanced_t *pid, bool enable) {
    pid->anti_windup = enable;
}

/**
 * @brief 设置PID D项低通滤波功能
 * 
 * @param pid 高级PID控制器结构体指针
 * @param enable 是否启用D项低通滤波
 * @param beta 滤波系数 (0-1)，越小滤波效果越强
 */
void pid_set_d_lpf(PID_Advanced_t *pid, bool enable, float beta) {
    pid->d_term_lpf = enable;
    if (beta > 0.0f && beta <= 1.0f) {
        pid->d_lpf_beta = beta;
    }
}

/**
 * @brief 重置高级PID控制器状态
 * 
 * @param pid 高级PID控制器结构体指针
 */
void pid_reset_advanced(PID_Advanced_t *pid) {
    pid->base_pid.integral = 0.0f;
    pid->base_pid.prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
}

/**
 * @brief 计算高级PID控制器输出，添加前馈控制等功能
 * 
 * @param pid 高级PID控制器结构体指针
 * @param setpoint 设定值
 * @param measurement 测量值
 * @param feedforward 前馈值
 * @param dt 采样时间间隔(秒)
 * @return float PID控制器输出
 */
float pid_compute_advanced(PID_Advanced_t *pid, float setpoint, float measurement, float feedforward, float dt) {
    float error = setpoint - measurement;
    
    // 比例项
    float p_term = pid->base_pid.kp * error;
    
    // 积分项 (使用积分限幅防止积分饱和)
    pid->base_pid.integral += error * dt;
    
    // 积分限幅
    if (pid->base_pid.integral > pid->integral_limit) {
        pid->base_pid.integral = pid->integral_limit;
    } else if (pid->base_pid.integral < -pid->integral_limit) {
        pid->base_pid.integral = -pid->integral_limit;
    }
    
    float i_term = pid->base_pid.ki * pid->base_pid.integral;
    
    // 微分项 (使用当前误差与上次误差的差值计算)
    float derivative = 0.0f;
    if (dt > 0.0f) {
        // 可以选择基于误差的微分或者基于测量值的微分
        // 使用基于测量值的微分可以避免设定值突变引起的微分spike
        // derivative = (error - pid->base_pid.prev_error) / dt; // 基于误差的微分
        derivative = (pid->prev_measurement - measurement) / dt; // 基于测量值的微分
    }
    
    // 微分项低通滤波
    if (pid->d_term_lpf) {
        // 一阶低通滤波: y(n) = beta * y(n-1) + (1-beta) * x(n)
        derivative = pid->d_lpf_beta * derivative + (1.0f - pid->d_lpf_beta) * derivative;
    }
    
    float d_term = pid->base_pid.kd * derivative;
    
    // 记录当前误差和测量值供下次计算使用
    pid->base_pid.prev_error = error;
    pid->prev_measurement = measurement;
    
    // 计算总输出
    float output = p_term + i_term + d_term;
    
    // 添加前馈项
    output += pid->kff * feedforward;
    
    // 对输出进行限幅
    float raw_output = output;
    if (output > pid->base_pid.output_limit) {
        output = pid->base_pid.output_limit;
    } else if (output < -pid->base_pid.output_limit) {
        output = -pid->base_pid.output_limit;
    }
    
    // 积分反饱和 (Anti-windup)
    if (pid->anti_windup && output != raw_output) {
        // 如果输出饱和，减少积分项以防饱和恶化
        pid->base_pid.integral = pid->base_pid.integral - (raw_output - output) * dt / pid->base_pid.ki;
        // 确保积分限幅
        if (pid->base_pid.integral > pid->integral_limit) {
            pid->base_pid.integral = pid->integral_limit;
        } else if (pid->base_pid.integral < -pid->integral_limit) {
            pid->base_pid.integral = -pid->integral_limit;
        }
    }
    
    return output;
}
