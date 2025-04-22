#ifndef ATTITUDE_CONTROL_H
#define ATTITUDE_CONTROL_H

#include "dart_control.h"
#include "pid_controller.h"

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

// 新增: 四元数结构体定义
typedef struct {
    float q0;    // w
    float q1;    // x
    float q2;    // y
    float q3;    // z
} Quaternion_t;

// 新增: 欧拉角速率结构体
typedef struct {
    float roll_rate;    // 横滚角速率 (度/秒)
    float pitch_rate;   // 俯仰角速率 (度/秒)
    float yaw_rate;     // 偏航角速率 (度/秒)
} AngularRates_t;

// 新增: 姿态控制器配置
typedef struct {
    // 姿态角PID
    PID_t roll_angle_pid;
    PID_t pitch_angle_pid;
    PID_t yaw_angle_pid;
    
    // 角速率PID
    PID_t roll_rate_pid;
    PID_t pitch_rate_pid;
    PID_t yaw_rate_pid;
    
    // 目标追踪PID
    PID_t x_target_pid;
    PID_t y_target_pid;
    
    // 控制分配参数
    float roll_control_weight;
    float pitch_control_weight;
    float yaw_control_weight;
    
    // 自适应控制参数
    bool adaptive_control_enabled;
    float adaptive_gain;
    float reference_model_tau;
    
} AttitudeControllerConfig_t;

/**
 * @brief 初始化飞镖系统
 */
void dart_system_init(void);

/**
 * @brief 初始化姿态控制器
 */
void attitude_control_init(void);

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

/**
 * @brief 获取姿态控制器配置
 * 
 * @return AttitudeControllerConfig_t* 姿态控制器配置指针
 */
AttitudeControllerConfig_t* get_attitude_controller_config(void);

/**
 * @brief 设置姿态控制器配置
 * 
 * @param config 姿态控制器配置
 */
void set_attitude_controller_config(AttitudeControllerConfig_t config);

/**
 * @brief 欧拉角转四元数
 * 
 * @param euler 欧拉角结构体
 * @param quaternion 输出的四元数结构体
 */
void euler_to_quaternion(const Attitude_t *euler, Quaternion_t *quaternion);

/**
 * @brief 四元数转欧拉角
 * 
 * @param quaternion 四元数结构体
 * @param euler 输出的欧拉角结构体
 */
void quaternion_to_euler(const Quaternion_t *quaternion, Attitude_t *euler);

/**
 * @brief 四元数乘法
 * 
 * @param q1 第一个四元数
 * @param q2 第二个四元数
 * @param result 结果四元数
 */
void quaternion_multiply(const Quaternion_t *q1, const Quaternion_t *q2, Quaternion_t *result);

/**
 * @brief 四元数共轭
 * 
 * @param q 输入四元数
 * @param result 结果四元数
 */
void quaternion_conjugate(const Quaternion_t *q, Quaternion_t *result);

/**
 * @brief 四元数标准化
 * 
 * @param q 输入/输出四元数
 */
void quaternion_normalize(Quaternion_t *q);

/**
 * @brief 计算两个姿态之间的误差四元数
 * 
 * @param q_current 当前姿态四元数
 * @param q_target 目标姿态四元数
 * @param q_error 输出的误差四元数
 */
void quaternion_error(const Quaternion_t *q_current, const Quaternion_t *q_target, Quaternion_t *q_error);

/**
 * @brief 创建表示姿态旋转的四元数
 * 
 * @param roll 横滚角(弧度)
 * @param pitch 俯仰角(弧度)
 * @param yaw 偏航角(弧度)
 * @param quaternion 输出的四元数
 */
void quaternion_from_euler_rad(float roll, float pitch, float yaw, Quaternion_t *quaternion);

/**
 * @brief 自动调整PID参数
 * 
 * @param target 目标数据
 * @param attitude 姿态数据
 * @param adaptive_gain 自适应增益
 */
void auto_tune_pid_parameters(const Target_t *target, const Attitude_t *attitude, float adaptive_gain);

#endif /* ATTITUDE_CONTROL_H */
