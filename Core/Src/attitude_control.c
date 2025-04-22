#include "dart_control.h"
#include "attitude_control.h"
#include "kalman_filter.h"
#include "mpu6050.h"
#include "flight_mode.h"
#include <math.h>
#include <string.h>

// 定义PI值 (M_PI可能在某些编译器未定义)
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

// 删除未使用的PID控制器实例
// static PID_t pid_roll;
// static PID_t pid_pitch;
// static PID_t pid_yaw;
// static PID_t pid_target_x;
// static PID_t pid_target_y;

// 删除未使用的系统状态变量
// static SystemState_t system_state = SYSTEM_INIT;

// 定义姿态控制器配置
static AttitudeControllerConfig_t attitude_controller_config;

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
    // 初始化IMU和传感器
    if (MPU6050_Init() != HAL_OK) {
        // 初始化失败处理
        while (1) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            HAL_Delay(200);
        }
    }
    
    // 初始化姿态控制器
    attitude_control_init();
    
    // 初始化飞行状态机
    flight_mode_init();
    
    // 其他初始化操作
    // ...
}

/**
 * @brief 根据目标位置计算舵机和电机控制量，使用更高级的控制策略
 * 
 * @param attitude 陀螺仪姿态数据
 * @param target 目标相对位置数据
 * @param control 输出的控制值
 */
void target_tracking(const Attitude_t *attitude, const Target_t *target, Control_t *control) {
    // 获取当前的飞行模式
    FlightMode_t current_mode = flight_mode_get();
    float desired_roll = 0.0f;
    float desired_pitch = 0.0f;
    MPU6050_Data_t *mpu_data = MPU6050_GetData();
    float roll_rate_sp = 0.0f;
    float pitch_rate_sp = 0.0f;
    float yaw_rate_sp = 0.0f;
    float roll_output = 0.0f;
    float pitch_output = 0.0f;
    float yaw_output = 0.0f;
    
    // 根据不同的飞行模式选择不同的控制策略
    switch (current_mode) {
        case MODE_TRACKING:
        case MODE_ATTACK:
            // 当处于追踪或攻击模式时，使用目标跟踪控制
            // 首先用目标位置计算期望的姿态角度
            
            if (target->detected) {
                // 使用目标位置PID控制器计算期望姿态角度
                // Y偏移映射到横滚角, X偏移映射到俯仰角
                desired_roll = -pid_compute(&attitude_controller_config.y_target_pid, 
                                         0.0f, target->y, 0.01f);
                desired_pitch = -pid_compute(&attitude_controller_config.x_target_pid, 
                                          0.0f, target->x, 0.01f);
                
                // 限制最大角度
                desired_roll = fmaxf(-ROLL_MAX, fminf(ROLL_MAX, desired_roll * 30.0f));
                desired_pitch = fmaxf(-PITCH_MAX, fminf(PITCH_MAX, desired_pitch * 20.0f));
            }
            
            // 将期望角度和当前姿态转换为四元数
            Quaternion_t current_q, desired_q, error_q;
            euler_to_quaternion(attitude, &current_q);
            
            // 创建期望姿态的四元数
            Attitude_t desired_attitude;
            desired_attitude.roll = desired_roll;
            desired_attitude.pitch = desired_pitch;
            desired_attitude.yaw = attitude->yaw; // 保持当前偏航角
            
            euler_to_quaternion(&desired_attitude, &desired_q);
            
            // 计算姿态误差
            quaternion_error(&current_q, &desired_q, &error_q);
            
            // 从误差四元数中提取角度误差
            Attitude_t angle_error;
            quaternion_to_euler(&error_q, &angle_error);
            
            // 使用角度误差和角速率进行级联PID控制
            // 获取当前角速率
            
            // 角度环PID -> 角速率设定值
            roll_rate_sp = pid_compute(
                &attitude_controller_config.roll_angle_pid, 
                0.0f, angle_error.roll, 0.01f);
            
            pitch_rate_sp = pid_compute(
                &attitude_controller_config.pitch_angle_pid, 
                0.0f, angle_error.pitch, 0.01f);
            
            yaw_rate_sp = pid_compute(
                &attitude_controller_config.yaw_angle_pid, 
                0.0f, angle_error.yaw, 0.01f);
            
            // 角速率环PID -> 控制输出
            roll_output = pid_compute(
                &attitude_controller_config.roll_rate_pid, 
                roll_rate_sp, mpu_data->gyro_x, 0.01f);
            
            pitch_output = pid_compute(
                &attitude_controller_config.pitch_rate_pid, 
                pitch_rate_sp, mpu_data->gyro_y, 0.01f);
            
            yaw_output = pid_compute(
                &attitude_controller_config.yaw_rate_pid, 
                yaw_rate_sp, mpu_data->gyro_z, 0.01f);
            
            // 根据控制权重分配控制输出
            roll_output *= attitude_controller_config.roll_control_weight;
            pitch_output *= attitude_controller_config.pitch_control_weight;
            yaw_output *= attitude_controller_config.yaw_control_weight;
            
            // 转换为舵机输出
            // 左舵面：俯仰向下为正，横滚向右为正
            // 右舵面：俯仰向下为正，横滚向左为正
            control->left_wing = (uint16_t)(SERVO_CENTER - pitch_output - roll_output);
            control->right_wing = (uint16_t)(SERVO_CENTER - pitch_output + roll_output);
            
            // 电机输出设置
            if (current_mode == MODE_ATTACK) {
                // 攻击模式使用最大推力
                control->thruster_1 = (uint16_t)(MOTOR_IDLE + (MOTOR_MAX_THRUST - MOTOR_IDLE) * 0.9f);
                control->thruster_2 = control->thruster_1;
            } else {
                // 追踪模式使用中等推力
                control->thruster_1 = (uint16_t)(MOTOR_IDLE + (MOTOR_MAX_THRUST - MOTOR_IDLE) * 0.7f);
                control->thruster_2 = control->thruster_1;
            }
            
            // 添加偏航控制（如果有的话）
            if (fabsf(yaw_output) > 1.0f) {
                // 使用差动推力控制偏航
                int16_t yaw_thrust_diff = (int16_t)(yaw_output * 0.1f * (MOTOR_MAX_THRUST - MOTOR_IDLE));
                control->thruster_1 += yaw_thrust_diff;
                control->thruster_2 -= yaw_thrust_diff;
            }
            
            // 如果启用了自适应控制，动态调整PID参数
            if (attitude_controller_config.adaptive_control_enabled) {
                auto_tune_pid_parameters(target, attitude, attitude_controller_config.adaptive_gain);
            }
            break;
            
        default:
            // 其他模式使用默认的姿态控制
            update_attitude_control(attitude, target, control);
            break;
    }
    
    // 确保控制输出在有效范围内
    control->left_wing = (uint16_t)fmaxf(SERVO_MIN_PULSE, fminf(SERVO_MAX_PULSE, (float)control->left_wing));
    control->right_wing = (uint16_t)fmaxf(SERVO_MIN_PULSE, fminf(SERVO_MAX_PULSE, (float)control->right_wing));
    control->thruster_1 = (uint16_t)fmaxf(MOTOR_IDLE, fminf(MOTOR_MAX_THRUST, (float)control->thruster_1));
    control->thruster_2 = (uint16_t)fmaxf(MOTOR_IDLE, fminf(MOTOR_MAX_THRUST, (float)control->thruster_2));
}

/**
 * @brief 更新姿态控制输出 (简化版本，主要功能由target_tracking实现)
 * 
 * @param attitude 陀螺仪姿态数据
 * @param target 目标相对位置数据
 * @param control 输出的控制值
 */
void update_attitude_control(const Attitude_t *attitude, const Target_t *target, Control_t *control) {
    float roll_setpoint = 0.0f;
    float pitch_setpoint = 0.0f;
    float yaw_setpoint = 0.0f;
    float dt = 0.01f; // 假设固定步长
    MPU6050_Data_t *mpu_data = MPU6050_GetData();
    
    // 如果检测到目标，计算期望的姿态角度以追踪目标
    if (target->detected) {
        // 简单的比例映射，将目标位置转换为期望角度
        roll_setpoint = -target->y * 30.0f;  // 目标Y轴偏差对应横滚角
        pitch_setpoint = -target->x * 20.0f; // 目标X轴偏差对应俯仰角
        
        // 限制最大角度
        roll_setpoint = fmaxf(-ROLL_MAX, fminf(ROLL_MAX, roll_setpoint));
        pitch_setpoint = fmaxf(-PITCH_MAX, fminf(PITCH_MAX, pitch_setpoint));
    }
    
    // 使用基本的角度环和角速率环级联PID控制
    // 角度环PID计算（外环）
    float roll_rate_setpoint = pid_compute(
        &attitude_controller_config.roll_angle_pid, 
        roll_setpoint, attitude->roll, dt);
    
    float pitch_rate_setpoint = pid_compute(
        &attitude_controller_config.pitch_angle_pid, 
        pitch_setpoint, attitude->pitch, dt);
    
    float yaw_rate_setpoint = pid_compute(
        &attitude_controller_config.yaw_angle_pid, 
        yaw_setpoint, attitude->yaw, dt);
    
    // 角速率环PID计算（内环）
    float roll_output = pid_compute(
        &attitude_controller_config.roll_rate_pid, 
        roll_rate_setpoint, mpu_data->gyro_x, dt);
    
    float pitch_output = pid_compute(
        &attitude_controller_config.pitch_rate_pid, 
        pitch_rate_setpoint, mpu_data->gyro_y, dt);
    
    float yaw_output = pid_compute(
        &attitude_controller_config.yaw_rate_pid, 
        yaw_rate_setpoint, mpu_data->gyro_z, dt);
    
    // 计算左右舵面控制输出
    control->left_wing = (uint16_t)(SERVO_CENTER - pitch_output - roll_output);
    control->right_wing = (uint16_t)(SERVO_CENTER - pitch_output + roll_output);
    
    // 根据目标跟踪状态调整推力
    if (target->detected) {
        // 当检测到目标时，使用较高的油门以加速接近
        control->thruster_1 = (uint16_t)(MOTOR_IDLE + (MOTOR_MAX_THRUST - MOTOR_IDLE) * 0.8f);
        control->thruster_2 = control->thruster_1;
    } else {
        // 未检测到目标时，使用巡航油门
        control->thruster_1 = (uint16_t)(MOTOR_IDLE + (MOTOR_MAX_THRUST - MOTOR_IDLE) * 0.5f);
        control->thruster_2 = control->thruster_1;
    }
    
    // 添加偏航控制的混合输出
    if (fabsf(yaw_output) > 1.0f) {
        // 使用差动推力来控制偏航
        int16_t yaw_thrust_diff = (int16_t)(yaw_output * 0.1f * (MOTOR_MAX_THRUST - MOTOR_IDLE));
        control->thruster_1 += yaw_thrust_diff;
        control->thruster_2 -= yaw_thrust_diff;
    }
    
    // 确保在有效范围内
    control->left_wing = (uint16_t)fmaxf(SERVO_MIN_PULSE, fminf(SERVO_MAX_PULSE, (float)control->left_wing));
    control->right_wing = (uint16_t)fmaxf(SERVO_MIN_PULSE, fminf(SERVO_MAX_PULSE, (float)control->right_wing));
    control->thruster_1 = (uint16_t)fmaxf(MOTOR_IDLE, fminf(MOTOR_MAX_THRUST, (float)control->thruster_1));
    control->thruster_2 = (uint16_t)fmaxf(MOTOR_IDLE, fminf(MOTOR_MAX_THRUST, (float)control->thruster_2));
}

/**
 * @brief 欧拉角转四元数
 * 
 * @param euler 欧拉角结构体
 * @param quaternion 输出的四元数结构体
 */
void euler_to_quaternion(const Attitude_t *euler, Quaternion_t *quaternion) {
    // 转换为弧度
    float roll_rad = euler->roll * DEG_TO_RAD;
    float pitch_rad = euler->pitch * DEG_TO_RAD;
    float yaw_rad = euler->yaw * DEG_TO_RAD;
    
    // 计算角度的一半的三角函数值
    float cr = cosf(roll_rad * 0.5f);
    float sr = sinf(roll_rad * 0.5f);
    float cp = cosf(pitch_rad * 0.5f);
    float sp = sinf(pitch_rad * 0.5f);
    float cy = cosf(yaw_rad * 0.5f);
    float sy = sinf(yaw_rad * 0.5f);
    
    // 计算四元数分量
    quaternion->q0 = cr * cp * cy + sr * sp * sy;
    quaternion->q1 = sr * cp * cy - cr * sp * sy;
    quaternion->q2 = cr * sp * cy + sr * cp * sy;
    quaternion->q3 = cr * cp * sy - sr * sp * cy;
    
    // 归一化四元数
    quaternion_normalize(quaternion);
}

/**
 * @brief 四元数转欧拉角
 * 
 * @param quaternion 四元数结构体
 * @param euler 输出的欧拉角结构体
 */
void quaternion_to_euler(const Quaternion_t *quaternion, Attitude_t *euler) {
    // 提取四元数分量
    float q0 = quaternion->q0;
    float q1 = quaternion->q1;
    float q2 = quaternion->q2;
    float q3 = quaternion->q3;
    
    // 计算欧拉角，采用ZYX顺序
    euler->roll = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * RAD_TO_DEG;
    
    // 防止万向锁
    float sinp = 2.0f * (q0*q2 - q3*q1);
    if (fabs(sinp) >= 1.0f) {
        euler->pitch = copysignf(90.0f, sinp);  // 正负90度
    } else {
        euler->pitch = asinf(sinp) * RAD_TO_DEG;
    }
    
    euler->yaw = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * RAD_TO_DEG;
    
    // 将偏航角规范化到[0, 360)
    if (euler->yaw < 0.0f) {
        euler->yaw += 360.0f;
    }
}

/**
 * @brief 四元数乘法
 * 
 * @param q1 第一个四元数
 * @param q2 第二个四元数
 * @param result 结果四元数
 */
void quaternion_multiply(const Quaternion_t *q1, const Quaternion_t *q2, Quaternion_t *result) {
    result->q0 = q1->q0*q2->q0 - q1->q1*q2->q1 - q1->q2*q2->q2 - q1->q3*q2->q3;
    result->q1 = q1->q0*q2->q1 + q1->q1*q2->q0 + q1->q2*q2->q3 - q1->q3*q2->q2;
    result->q2 = q1->q0*q2->q2 - q1->q1*q2->q3 + q1->q2*q2->q0 + q1->q3*q2->q1;
    result->q3 = q1->q0*q2->q3 + q1->q1*q2->q2 - q1->q2*q2->q1 + q1->q3*q2->q0;
}

/**
 * @brief 四元数共轭
 * 
 * @param q 输入四元数
 * @param result 结果四元数
 */
void quaternion_conjugate(const Quaternion_t *q, Quaternion_t *result) {
    result->q0 = q->q0;
    result->q1 = -q->q1;
    result->q2 = -q->q2;
    result->q3 = -q->q3;
}

/**
 * @brief 四元数标准化
 * 
 * @param q 输入/输出四元数
 */
void quaternion_normalize(Quaternion_t *q) {
    float norm = sqrtf(q->q0*q->q0 + q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3);
    
    // 防止除零
    if (norm < 1e-10f) {
        q->q0 = 1.0f;
        q->q1 = q->q2 = q->q3 = 0.0f;
        return;
    }
    
    // 归一化
    float inv_norm = 1.0f / norm;
    q->q0 *= inv_norm;
    q->q1 *= inv_norm;
    q->q2 *= inv_norm;
    q->q3 *= inv_norm;
}

/**
 * @brief 计算两个姿态之间的误差四元数
 * 
 * @param q_current 当前姿态四元数
 * @param q_target 目标姿态四元数
 * @param q_error 输出的误差四元数
 */
void quaternion_error(const Quaternion_t *q_current, const Quaternion_t *q_target, Quaternion_t *q_error) {
    Quaternion_t q_current_conj;
    quaternion_conjugate(q_current, &q_current_conj);
    quaternion_multiply(q_target, &q_current_conj, q_error);
}

/**
 * @brief 自动调整PID参数
 * 
 * @param target 目标数据
 * @param attitude 姿态数据
 * @param adaptive_gain 自适应增益
 */
void auto_tune_pid_parameters(const Target_t *target, const Attitude_t *attitude, float adaptive_gain) {
    // 仅当启用自适应控制且检测到目标时进行调整
    if (!attitude_controller_config.adaptive_control_enabled || !target->detected) {
        return;
    }
    
    // 根据目标偏差和姿态变化调整PID参数
    // 这是一个简化的自适应控制实现，实际应用中需要更复杂的算法
    
    // 计算目标偏差的绝对值
    float abs_x_error = fabsf(target->x);
    float abs_y_error = fabsf(target->y);
    
    // 如果目标接近中心，增强P参数以提高精度
    if (abs_x_error < 0.2f && abs_y_error < 0.2f) {
        attitude_controller_config.pitch_angle_pid.kp += adaptive_gain * 0.01f;
        attitude_controller_config.roll_angle_pid.kp += adaptive_gain * 0.01f;
        
        // 限制最大增益
        if (attitude_controller_config.pitch_angle_pid.kp > PITCH_KP * 1.5f) {
            attitude_controller_config.pitch_angle_pid.kp = PITCH_KP * 1.5f;
        }
        if (attitude_controller_config.roll_angle_pid.kp > ROLL_KP * 1.5f) {
            attitude_controller_config.roll_angle_pid.kp = ROLL_KP * 1.5f;
        }
    }
    // 如果目标偏差较大，增强D参数以避免过冲
    else if (abs_x_error > 0.6f || abs_y_error > 0.6f) {
        attitude_controller_config.pitch_angle_pid.kd += adaptive_gain * 0.005f;
        attitude_controller_config.roll_angle_pid.kd += adaptive_gain * 0.005f;
        
        // 限制最大D增益
        if (attitude_controller_config.pitch_angle_pid.kd > PITCH_KD * 1.5f) {
            attitude_controller_config.pitch_angle_pid.kd = PITCH_KD * 1.5f;
        }
        if (attitude_controller_config.roll_angle_pid.kd > ROLL_KD * 1.5f) {
            attitude_controller_config.roll_angle_pid.kd = ROLL_KD * 1.5f;
        }
    }
    // 如果目标在中等距离，恢复默认参数
    else {
        // 缓慢恢复到默认参数
        attitude_controller_config.pitch_angle_pid.kp = 
            0.99f * attitude_controller_config.pitch_angle_pid.kp + 0.01f * PITCH_KP;
        attitude_controller_config.roll_angle_pid.kp = 
            0.99f * attitude_controller_config.roll_angle_pid.kp + 0.01f * ROLL_KP;
        attitude_controller_config.pitch_angle_pid.kd = 
            0.99f * attitude_controller_config.pitch_angle_pid.kd + 0.01f * PITCH_KD;
        attitude_controller_config.roll_angle_pid.kd = 
            0.99f * attitude_controller_config.roll_angle_pid.kd + 0.01f * ROLL_KD;
    }
}

/**
 * @brief 初始化姿态控制器
 */
void attitude_control_init(void) {
    // 初始化角度环PID
    pid_init(&attitude_controller_config.roll_angle_pid, 
                      ROLL_KP, ROLL_KI, ROLL_KD, ROLL_MAX);
    pid_init(&attitude_controller_config.pitch_angle_pid, 
                      PITCH_KP, PITCH_KI, PITCH_KD, PITCH_MAX);
    pid_init(&attitude_controller_config.yaw_angle_pid, 
                      YAW_KP, YAW_KI, YAW_KD, YAW_MAX);
    
    // 初始化角速率环PID，使用更快的响应参数
    pid_init(&attitude_controller_config.roll_rate_pid, 
                      2.5f, 0.1f, 0.05f, 200.0f);
    pid_init(&attitude_controller_config.pitch_rate_pid, 
                      2.0f, 0.1f, 0.05f, 200.0f);
    pid_init(&attitude_controller_config.yaw_rate_pid, 
                      1.5f, 0.01f, 0.1f, 200.0f);
    
    // 初始化目标追踪PID
    pid_init(&attitude_controller_config.x_target_pid, 
                      TARGET_KP, TARGET_KI, TARGET_KD, TARGET_MAX);
    pid_init(&attitude_controller_config.y_target_pid, 
                      TARGET_KP, TARGET_KI, TARGET_KD, TARGET_MAX);
    
    // 设置控制分配权重
    attitude_controller_config.roll_control_weight = 1.0f;
    attitude_controller_config.pitch_control_weight = 1.0f;
    attitude_controller_config.yaw_control_weight = 0.5f;
    
    // 设置自适应控制参数
    attitude_controller_config.adaptive_control_enabled = true;
    attitude_controller_config.adaptive_gain = 0.01f;
    attitude_controller_config.reference_model_tau = 0.5f;
}

/**
 * @brief 获取姿态控制器配置
 * 
 * @return AttitudeControllerConfig_t* 姿态控制器配置指针
 */
AttitudeControllerConfig_t* get_attitude_controller_config(void) {
    return &attitude_controller_config;
}

/**
 * @brief 设置姿态控制器配置
 * 
 * @param config 姿态控制器配置
 */
void set_attitude_controller_config(AttitudeControllerConfig_t config) {
    attitude_controller_config = config;
}
