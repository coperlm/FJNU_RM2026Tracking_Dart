#ifndef DART_CONTROL_H
#define DART_CONTROL_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// PWM控制参数定义
#define PWM_FREQ_HZ      50      // 舵机标准PWM频率为50Hz
#define SERVO_MIN_PULSE  1000    // 舵机最小脉宽(微秒)
#define SERVO_MAX_PULSE  2000    // 舵机最大脉宽(微秒)
#define MOTOR_MIN_PULSE  1000    // 电机最小脉宽(微秒)
#define MOTOR_MAX_PULSE  2000    // 电机最大脉宽(微秒)

// 舵机通道定义
#define SERVO_LEFT_WING  0       // 左侧舵机通道
#define SERVO_RIGHT_WING 1       // 右侧舵机通道
#define MOTOR_THRUSTER_1 2       // 涵道电机通道1
#define MOTOR_THRUSTER_2 3       // 涵道电机通道2

// 硬件定时器通道定义
#define SERVO_LEFT_CHANNEL  TIM_CHANNEL_1  // 左侧舵机使用的定时器通道
#define SERVO_RIGHT_CHANNEL TIM_CHANNEL_2  // 右侧舵机使用的定时器通道
#define MOTOR_CHANNEL_1     TIM_CHANNEL_1  // 涵道电机1使用的定时器通道
#define MOTOR_CHANNEL_2     TIM_CHANNEL_2  // 涵道电机2使用的定时器通道

// PID控制参数结构
typedef struct {
    float kp;                    // 比例系数
    float ki;                    // 积分系数
    float kd;                    // 微分系数
    float integral;              // 积分项
    float prev_error;            // 上一次误差
    float output_limit;          // 输出限幅
} PID_t;

// 姿态数据结构
typedef struct {
    float roll;                  // 横滚角(度)
    float pitch;                 // 俯仰角(度)
    float yaw;                   // 偏航角(度)
} Attitude_t;

// 目标数据结构
typedef struct {
    float x;                     // X轴相对位置(-1.0到1.0)
    float y;                     // Y轴相对位置(-1.0到1.0)
    bool detected;               // 是否检测到目标
} Target_t;

// 控制输出结构
typedef struct {
    uint16_t left_wing;          // 左翼舵机PWM值
    uint16_t right_wing;         // 右翼舵机PWM值
    uint16_t thruster_1;         // 涵道电机1 PWM值
    uint16_t thruster_2;         // 涵道电机2 PWM值
} Control_t;

// 系统状态枚举
typedef enum {
    SYSTEM_INIT,                 // 初始化状态
    SYSTEM_IDLE,                 // 空闲状态
    SYSTEM_TARGETING,            // 目标追踪状态
    SYSTEM_ATTACK                // 攻击状态
} SystemState_t;

// 功能初始化函数
void dart_system_init(void);

// 姿态控制函数
void update_attitude_control(const Attitude_t *attitude, const Target_t *target, Control_t *control);

// PWM控制函数
void pwm_init(void);
void pwm_set_servo(uint8_t channel, uint16_t pulse_width_us);
void pwm_set_motor(uint8_t channel, uint16_t pulse_width_us);

// 目标追踪函数
void target_tracking(const Attitude_t *attitude, const Target_t *target, Control_t *control);

// PID控制器初始化和更新
void pid_init(PID_t *pid, float kp, float ki, float kd, float output_limit);
float pid_compute(PID_t *pid, float setpoint, float measurement, float dt);

// 姿态模拟函数（实际系统应替换为传感器读取）
void simulate_gyro(Attitude_t *attitude);

// 目标模拟函数（实际系统应替换为视觉系统或其他传感器）
void simulate_target(Target_t *target, const Attitude_t *attitude);

#endif /* DART_CONTROL_H */
