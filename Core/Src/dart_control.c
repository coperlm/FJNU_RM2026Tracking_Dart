#include "dart_control.h"
#include "mpu6050.h"
#include "kalman_filter.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* PX4风格的飞控算法 - 姿态控制器 */
typedef struct {
    float roll_setpoint;    // 横滚角目标值
    float pitch_setpoint;   // 俯仰角目标值
    float yaw_setpoint;     // 偏航角目标值
    
    // 角度环PID
    PID_t roll_angle_pid;
    PID_t pitch_angle_pid;
    PID_t yaw_angle_pid;
    
    // 角速度环PID
    PID_t roll_rate_pid;
    PID_t pitch_rate_pid;
    PID_t yaw_rate_pid;
    
    // 输出限幅
    float roll_output_limit;
    float pitch_output_limit;
    float yaw_output_limit;
} AttitudeController_t;

// 固定翼特定控制参数
typedef struct {
    float airspeed;           // 空速
    float throttle_min;       // 最小油门
    float throttle_cruise;    // 巡航油门
    float throttle_max;       // 最大油门
    
    // L1控制器参数（用于路径跟踪）
    float L1_period;          // L1周期
    float L1_damping;         // L1阻尼
    
    // TECS参数（总能量控制系统）
    float TECS_timeConst;     // TECS时间常数
    float TECS_thrDamp;       // TECS油门阻尼
    float TECS_pitchDamp;     // TECS俯仰阻尼
    
    // 翼面控制限制
    float roll_limit_rad;     // 横滚角限制(弧度)
    float pitch_limit_rad;    // 俯仰角限制(弧度)
} FixedWingParams_t;

static AttitudeController_t attitude_controller;
static FixedWingParams_t fw_params;

/**
 * @brief 模拟视觉目标数据
 * 在实际应用中，应替换为实际的视觉传感器或其他目标探测系统
 * 
 * @param target 输出的目标数据结构体指针
 * @param attitude 当前姿态数据，用于计算目标相对位置
 */
void simulate_target(Target_t *target, const Attitude_t *attitude) {
    static float target_distance = 10.0f; // 初始距离10米
    static float target_x_offset = 0.3f;  // 初始X偏移
    static float target_y_offset = 0.2f;  // 初始Y偏移
    
    // 模拟飞镖接近目标的过程，距离逐渐减小
    target_distance -= 0.05f;
    if (target_distance < 0.5f) {
        target_distance = 10.0f; // 重置距离，模拟新的目标
        target_x_offset = ((float)rand() / RAND_MAX - 0.5f) * 0.8f;
        target_y_offset = ((float)rand() / RAND_MAX - 0.5f) * 0.8f;
    }
    
    // 根据姿态变化影响目标的相对位置
    target->x = target_x_offset + attitude->roll / 90.0f;
    target->y = target_y_offset - attitude->pitch / 90.0f;
    
    // 确保目标坐标在有效范围内 [-1, 1]
    target->x = fmaxf(-1.0f, fminf(1.0f, target->x));
    target->y = fmaxf(-1.0f, fminf(1.0f, target->y));
    
    // 当距离较远时，目标可能不在视野内
    target->detected = (target_distance < 9.0f);
}

/* 初始化固定翼控制器参数 */
static void fixed_wing_controller_init(void) {
    // 从attitude_control.h获取PID参数定义
    #include "attitude_control.h"
    
    // 初始化固定翼参数
    fw_params.throttle_min = 0.1f;
    fw_params.throttle_cruise = 0.5f;
    fw_params.throttle_max = 0.9f;
    
    fw_params.L1_period = 25.0f;
    fw_params.L1_damping = 0.75f;
    
    fw_params.TECS_timeConst = 5.0f;
    fw_params.TECS_thrDamp = 0.5f;
    fw_params.TECS_pitchDamp = 0.5f;
    
    fw_params.roll_limit_rad = 45.0f * (float)M_PI / 180.0f;
    fw_params.pitch_limit_rad = 30.0f * (float)M_PI / 180.0f;
    
    // 初始化姿态控制器
    pid_init(&attitude_controller.roll_angle_pid, ROLL_KP, ROLL_KI, ROLL_KD, ROLL_MAX);
    pid_init(&attitude_controller.pitch_angle_pid, PITCH_KP, PITCH_KI, PITCH_KD, PITCH_MAX);
    pid_init(&attitude_controller.yaw_angle_pid, YAW_KP, YAW_KI, YAW_KD, YAW_MAX);
    
    // 角速率环PID (更快的响应)
    pid_init(&attitude_controller.roll_rate_pid, 2.5f, 0.1f, 0.05f, 200.0f);
    pid_init(&attitude_controller.pitch_rate_pid, 2.0f, 0.1f, 0.05f, 200.0f);
    pid_init(&attitude_controller.yaw_rate_pid, 1.5f, 0.01f, 0.1f, 200.0f);
    
    attitude_controller.roll_output_limit = SERVO_TRAVEL;
    attitude_controller.pitch_output_limit = SERVO_TRAVEL;
    attitude_controller.yaw_output_limit = SERVO_TRAVEL;
}

/**
 * @brief 使用真实MPU6050替代模拟陀螺仪数据
 * 
 * @param attitude 输出的姿态数据结构体指针
 */
void simulate_gyro(Attitude_t *attitude) {
    // 直接调用我们新创建模块中的姿态读取函数
    read_mpu6050_attitude(attitude);
}

/**
 * @brief 飞镖系统初始化函数
 * 添加MPU6050初始化和固定翼控制器初始化
 */
void dart_system_init(void) {
    // 初始化MPU6050
    if (MPU6050_Init() != HAL_OK) {
        // 初始化失败处理
        while (1) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // 假设PC13是LED引脚，用于指示错误
            HAL_Delay(200);
        }
    }
    
    // 初始化固定翼控制器
    fixed_wing_controller_init();
    
    // 初始化PID控制器等其他功能
    // ...
}

/**
 * @brief PX4风格的姿态控制器更新
 * 采用级联PID控制结构，包含外环姿态角PID和内环角速率PID
 * 
 * @param attitude 当前姿态数据
 * @param target 目标相对位置数据
 * @param control 输出的控制值
 */
void update_attitude_control(const Attitude_t *attitude, const Target_t *target, Control_t *control) {
    float roll_setpoint = 0.0f;
    float pitch_setpoint = 0.0f;
    float yaw_setpoint = 0.0f;
    float dt = DT;
    MPU6050_Data_t *mpu_data = MPU6050_GetData(); // 获取最新的MPU6050数据用于角速率控制
    
    // 如果检测到目标，计算期望的姿态角度以追踪目标
    if (target->detected) {
        // 简单的比例映射，将目标位置转换为期望角度
        roll_setpoint = -target->y * 30.0f;  // 目标Y轴偏差对应横滚角
        pitch_setpoint = -target->x * 20.0f; // 目标X轴偏差对应俯仰角
        
        // 限制最大角度
        roll_setpoint = fmaxf(-ROLL_MAX, fminf(ROLL_MAX, roll_setpoint));
        pitch_setpoint = fmaxf(-PITCH_MAX, fminf(PITCH_MAX, pitch_setpoint));
    }
    
    // 角度环PID计算（外环）
    float roll_rate_setpoint = pid_compute(&attitude_controller.roll_angle_pid, roll_setpoint, attitude->roll, dt);
    float pitch_rate_setpoint = pid_compute(&attitude_controller.pitch_angle_pid, pitch_setpoint, attitude->pitch, dt);
    float yaw_rate_setpoint = pid_compute(&attitude_controller.yaw_angle_pid, yaw_setpoint, attitude->yaw, dt);
    
    // 角速率环PID计算（内环）
    float roll_output = pid_compute(&attitude_controller.roll_rate_pid, roll_rate_setpoint, mpu_data->gyro_x, dt);
    float pitch_output = pid_compute(&attitude_controller.pitch_rate_pid, pitch_rate_setpoint, mpu_data->gyro_y, dt);
    float yaw_output = pid_compute(&attitude_controller.yaw_rate_pid, yaw_rate_setpoint, mpu_data->gyro_z, dt);
    
    // 限制输出范围
    roll_output = fmaxf(-attitude_controller.roll_output_limit, fminf(attitude_controller.roll_output_limit, roll_output));
    pitch_output = fmaxf(-attitude_controller.pitch_output_limit, fminf(attitude_controller.pitch_output_limit, pitch_output));
    yaw_output = fmaxf(-attitude_controller.yaw_output_limit, fminf(attitude_controller.yaw_output_limit, yaw_output));
    
    // 计算左右舵面控制输出：差动配置（反向）用于横滚，同向用于俯仰
    // 左舵面（注意：PWM值与控制方向关系取决于实际安装方向）
    control->left_wing = (uint16_t)(SERVO_CENTER - pitch_output - roll_output);
    
    // 右舵面
    control->right_wing = (uint16_t)(SERVO_CENTER - pitch_output + roll_output);
    
    // 限制舵机PWM范围
    control->left_wing = (uint16_t)fmaxf(SERVO_MIN_PULSE, fminf(SERVO_MAX_PULSE, (float)control->left_wing));
    control->right_wing = (uint16_t)fmaxf(SERVO_MIN_PULSE, fminf(SERVO_MAX_PULSE, (float)control->right_wing));
    
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
    
    // 添加偏航控制的混合输出（如果有额外的控制面或推力差异）
    if (fabsf(yaw_output) > 1.0f) {
        // 使用差动推力来控制偏航
        int16_t yaw_thrust_diff = (int16_t)(yaw_output * 0.1f * (MOTOR_MAX_THRUST - MOTOR_IDLE));
        control->thruster_1 += yaw_thrust_diff;
        control->thruster_2 -= yaw_thrust_diff;
        
        // 确保在有效范围内
        control->thruster_1 = (uint16_t)fmaxf(MOTOR_IDLE, fminf(MOTOR_MAX_THRUST, (float)control->thruster_1));
        control->thruster_2 = (uint16_t)fmaxf(MOTOR_IDLE, fminf(MOTOR_MAX_THRUST, (float)control->thruster_2));
    }
}
