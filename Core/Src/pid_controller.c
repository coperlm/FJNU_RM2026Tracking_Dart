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
    pid->kff = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->output_limit = output_limit;
    pid->integral_limit = output_limit;
    pid->d_lpf_beta = 0.7f;
    pid->anti_windup = false;
    pid->d_term_lpf = false;
    pid->last_output = 0.0f;
    pid->last_calc_time = HAL_GetTick();
}

/**
 * @brief 初始化高级PID控制器参数
 * 
 * @param pid PID控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param kff 前馈系数
 * @param output_limit 输出限幅值
 * @param integral_limit 积分限幅值
 */
void pid_init_advanced(PID_t *pid, float kp, float ki, float kd, float kff, float output_limit, float integral_limit) {
    pid_init(pid, kp, ki, kd, output_limit);
    pid->kff = kff;
    pid->integral_limit = integral_limit;
    pid->anti_windup = true;
    pid->d_term_lpf = true;
}

/**
 * @brief 设置PID积分反饱和功能
 * 
 * @param pid PID控制器结构体指针
 * @param enable 是否启用积分反饱和
 */
void pid_set_anti_windup(PID_t *pid, bool enable) {
    pid->anti_windup = enable;
}

/**
 * @brief 设置PID D项低通滤波功能
 * 
 * @param pid PID控制器结构体指针
 * @param enable 是否启用D项低通滤波
 * @param beta 滤波系数 (0-1)，越小滤波效果越强
 */
void pid_set_d_lpf(PID_t *pid, bool enable, float beta) {
    pid->d_term_lpf = enable;
    if (beta > 0.0f && beta <= 1.0f) {
        pid->d_lpf_beta = beta;
    }
}

/**
 * @brief 重置PID控制器状态
 * 
 * @param pid PID控制器结构体指针
 */
void pid_reset(PID_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->last_output = 0.0f;
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
    return pid_compute_advanced(pid, setpoint, measurement, 0.0f, dt);
}

/**
 * @brief 计算高级PID控制器输出
 * 
 * @param pid PID控制器结构体指针
 * @param setpoint 设定值
 * @param measurement 测量值
 * @param feedforward 前馈值
 * @param dt 采样时间间隔(秒)
 * @return float PID控制器输出
 */
float pid_compute_advanced(PID_t *pid, float setpoint, float measurement, float feedforward, float dt) {
    // 如果dt过小，可能导致计算不稳定
    if (dt < 0.0001f) {
        dt = 0.0001f;
    }
    
    // 更新计算时间（用于自适应dt）
    pid->last_calc_time = HAL_GetTick();
    
    // 计算误差
    float error = setpoint - measurement;
    
    // 比例项
    float p_term = pid->kp * error;
    
    // 积分项
    pid->integral += error * dt;
    
    // 积分限幅
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    
    float i_term = pid->ki * pid->integral;
    
    // 微分项 (避免突变引起的微分爆震)
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - pid->prev_error) / dt;
    }
    
    // 微分项低通滤波
    if (pid->d_term_lpf) {
        // 一阶低通滤波: y(n) = beta * y(n-1) + (1-beta) * x(n)
        derivative = pid->d_lpf_beta * pid->prev_derivative + (1.0f - pid->d_lpf_beta) * derivative;
        pid->prev_derivative = derivative;
    }
    
    float d_term = pid->kd * derivative;
    
    // 前馈项
    float ff_term = pid->kff * feedforward;
    
    // 记录当前误差供下次计算使用
    pid->prev_error = error;
    
    // 计算总输出
    float output = p_term + i_term + d_term + ff_term;
    
    // 输出限幅前保存当前输出
    float raw_output = output;
    
    // 对输出进行限幅
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }
    
    // 积分反饱和 (Anti-windup)
    if (pid->anti_windup && output != raw_output) {
        // 如果输出饱和，减少积分项以防饱和恶化
        // 这里使用back-calculation方法
        pid->integral = pid->integral - (raw_output - output) * dt / pid->ki;
    }
    
    // 保存输出用于诊断
    pid->last_output = output;
    
    return output;
}
