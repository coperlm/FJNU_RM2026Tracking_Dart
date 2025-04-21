#include "dart_control.h"
#include "attitude_control.h"
#include "kalman_filter.h"
#include "mpu6050.h"
#include "flight_mode.h"
#include <math.h>
#include <string.h>

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
    
    // 根据不同的飞行模式选择不同的控制策略
    switch (current_mode) {
        case MODE_TRACKING:
        case MODE_ATTACK:
            // 当处于追踪或攻击模式时，使用目标跟踪控制
            // 首先用目标位置计算期望的姿态角度
            float desired_roll = 0.0f;
            float desired_pitch = 0.0f;
            
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
            MPU6050_Data_t *mpu_data = MPU6050_GetData();
            
            // 角度环PID -> 角速率设定值
            float roll_rate_sp = pid_compute_advanced(
                &attitude_controller_config.roll_angle_pid, 
                0.0f, angle_error.roll, 0.0f, 0.01f);
            
            float pitch_rate_sp = pid_compute_advanced(
                &attitude_controller_config.pitch_angle_pid, 
                0.0f, angle_error.pitch, 0.0f, 0.01f);
            
            float yaw_rate_sp = pid_compute_advanced(
                &attitude_controller_config.yaw_angle_pid, 
                0.0f, angle_error.yaw, 0.0f, 0.01f);
            
            // 角速率环PID -> 控制输出
            float roll_output = pid_compute_advanced(
                &attitude_controller_config.roll_rate_pid, 
                roll_rate_sp, mpu_data->gyro_x, 0.0f, 0.01f);
            
            float pitch_output = pid_compute_advanced(
                &attitude_controller_config.pitch_rate_pid, 
                pitch_rate_sp, mpu_data->gyro_y, 0.0f, 0.01f);
            
            float yaw_output = pid_compute_advanced(
                &attitude_controller_config.yaw_rate_pid, 
                yaw_rate_sp, mpu_data->gyro_z, 0.0f, 0.01f);
            
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
    control->thruster_1 = (uint16_t)fmaxf(MOTOR_MIN_PULSE, fminf(MOTOR_MAX_PULSE, (float)control->thruster_1));
    control->thruster_2 = (uint16_t)fmaxf(MOTOR_MIN_PULSE, fminf(MOTOR_MAX_PULSE, (float)control->thruster_2));
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
