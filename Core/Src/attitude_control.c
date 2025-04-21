#include "dart_control.h"
#include <math.h>

// PID控制器实例
static PID_t pid_roll;
static PID_t pid_pitch;
static PID_t pid_yaw;
static PID_t pid_target_x;
static PID_t pid_target_y;

// 系统状态
static SystemState_t system_state = SYSTEM_INIT;

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
void dart_system_init(void) {
    // 初始化PWM控制
    pwm_init();
    
    // 初始化PID控制器
    pid_init(&pid_roll, ROLL_KP, ROLL_KI, ROLL_KD, ROLL_MAX);
    pid_init(&pid_pitch, PITCH_KP, PITCH_KI, PITCH_KD, PITCH_MAX);
    pid_init(&pid_yaw, YAW_KP, YAW_KI, YAW_KD, YAW_MAX);
    pid_init(&pid_target_x, TARGET_KP, TARGET_KI, TARGET_KD, TARGET_MAX);
    pid_init(&pid_target_y, TARGET_KP, TARGET_KI, TARGET_KD, TARGET_MAX);
    
    // 设置舵机中立位置
    pwm_set_servo(SERVO_LEFT_WING, SERVO_CENTER);
    pwm_set_servo(SERVO_RIGHT_WING, SERVO_CENTER);
    
    // 设置电机初始值(低速)
    pwm_set_motor(MOTOR_THRUSTER, MOTOR_IDLE);
    
    // 更新系统状态
    system_state = SYSTEM_IDLE;
}

/**
 * @brief 根据目标位置计算舵机和电机控制量
 * 
 * @param attitude 陀螺仪姿态数据
 * @param target 目标相对位置数据
 * @param control 输出的控制值
 */
void target_tracking(const Attitude_t *attitude, const Target_t *target, Control_t *control) {
    float dt = 0.01f; // 假设采样时间为10ms
    
    // 如果检测到目标，计算目标修正量
    if (target->detected) {
        // 通过PID计算目标位置的修正量
        // 目标应该在中心(x=0, y=0)
        // 注意：不需要保存x_correction，直接使用pid_target_x的prev_error
        pid_compute(&pid_target_x, 0.0f, target->x, dt);
        pid_compute(&pid_target_y, 0.0f, target->y, dt);
        
        // 更新系统状态为目标追踪
        system_state = SYSTEM_TARGETING;
        
        // 如果目标位置接近中心，进入攻击状态
        if (fabs(target->x) < 0.1f && fabs(target->y) < 0.1f) {
            system_state = SYSTEM_ATTACK;
        }
    } else {
        // 没有检测到目标，保持姿态稳定
        system_state = SYSTEM_IDLE;
    }
    
    // 根据系统状态调整电机输出
    switch (system_state) {
        case SYSTEM_IDLE:
            control->thruster = MOTOR_IDLE;
            break;
            
        case SYSTEM_TARGETING:
            control->thruster = (MOTOR_IDLE + MOTOR_MAX_THRUST) / 2;
            break;
            
        case SYSTEM_ATTACK:
            control->thruster = MOTOR_MAX_THRUST;
            break;
            
        default:
            control->thruster = MOTOR_IDLE;
    }
}

/**
 * @brief 更新姿态控制输出
 * 
 * @param attitude 陀螺仪姿态数据
 * @param target 目标相对位置数据
 * @param control 输出的控制值
 */
void update_attitude_control(const Attitude_t *attitude, const Target_t *target, Control_t *control) {
    float dt = 0.01f; // 假设采样时间为10ms
    float roll_correction, pitch_correction;
    float y_correction = 0.0f;
    
    // 通过PID计算姿态修正量 - 期望保持水平姿态(roll=0, pitch=0)
    roll_correction = pid_compute(&pid_roll, 0.0f, attitude->roll, dt);
    pitch_correction = pid_compute(&pid_pitch, 0.0f, attitude->pitch, dt);
    
    // 如果检测到目标，更新目标跟踪状态和修正量
    if (target->detected) {
        // 通过PID计算目标位置的修正量
        // 直接使用pid计算更新积分和历史误差，结果暂时不使用，这样避免警告
        pid_compute(&pid_target_x, 0.0f, target->x, dt);
        y_correction = pid_compute(&pid_target_y, 0.0f, target->y, dt);
        
        // 目标跟踪状态更新
        target_tracking(attitude, target, control);
    } else {
        control->thruster = MOTOR_IDLE;
    }
    
    // 计算最终的控制输出
    // 左侧舵机: 中心值 + 横滚修正 - 目标Y方向修正
    // 右侧舵机: 中心值 - 横滚修正 - 目标Y方向修正
    // 俯仰通过同时调整两侧舵机实现
    control->left_wing = SERVO_CENTER + (int16_t)(roll_correction * SERVO_TRAVEL / ROLL_MAX) 
                       - (int16_t)(y_correction * SERVO_TRAVEL) 
                       + (int16_t)(pitch_correction * SERVO_TRAVEL / PITCH_MAX);
                       
    control->right_wing = SERVO_CENTER - (int16_t)(roll_correction * SERVO_TRAVEL / ROLL_MAX) 
                        - (int16_t)(y_correction * SERVO_TRAVEL) 
                        + (int16_t)(pitch_correction * SERVO_TRAVEL / PITCH_MAX);
    
    // 输出控制信号到PWM
    pwm_set_servo(SERVO_LEFT_WING, control->left_wing);
    pwm_set_servo(SERVO_RIGHT_WING, control->right_wing);
    pwm_set_motor(MOTOR_THRUSTER, control->thruster);
}
