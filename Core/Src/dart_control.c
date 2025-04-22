#include "dart_control.h"
#include "mpu6050.h"
#include "kalman_filter.h"
#include "flight_mode.h"
#include "i2c.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

// 定义PI值 (M_PI可能在某些编译器未定义)
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 外部变量声明 */
extern MPU6050_t MPU6050;
extern I2C_HandleTypeDef hi2c1;

/* 静态函数声明 */
// 移除未使用的函数声明
// static void fixed_wing_controller_init(void);
static void search_pattern(const Attitude_t *attitude, Control_t *control, float dt);
static void attack_control(const Attitude_t *attitude, const Target_t *target, Control_t *control);
static void return_to_base(const Attitude_t *attitude, Control_t *control, float dt);
static void failsafe_control(const Attitude_t *attitude, Control_t *control);

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

//static AttitudeController_t attitude_controller;
// 移除未使用的变量
// static FixedWingParams_t fw_params;

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

/**
 * @brief 使用真实MPU6050替代模拟陀螺仪数据
 * 
 * @param attitude 输出的姿态数据结构体指针
 */
void simulate_gyro(Attitude_t *attitude) {
    // 直接调用我们新创建模块中的姿态读取函数
    read_mpu6050_attitude(attitude);
}

// 移除dart_system_init函数，由attitude_control.c提供实现
// 移除update_attitude_control函数，由attitude_control.c提供实现

/**
 * @brief  飞行控制主循环，根据当前飞行模式执行相应控制逻辑
 * @param  mode: 当前飞行模式
 * @retval None
 */
void dart_control_update(FlightMode_t mode)
{
    static Attitude_t current_attitude = {0};
    static Target_t current_target = {0};
    static Control_t control_output = {0};
    static uint32_t last_update_time = 0;
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_update_time) / 1000.0f;  // 转换为秒
    
    // 防止首次运行时dt过大
    if (dt > 0.1f || dt <= 0) {
        dt = 0.01f;  // 默认使用10ms作为更新周期
    }
    
    last_update_time = current_time;
    
    // 更新姿态数据（从MPU6050获取）
    MPU6050_Read_All(&hi2c1, &MPU6050);
    
    // 更新卡尔曼滤波后的姿态数据
    current_attitude.roll = MPU6050.KalmanAngleX;
    current_attitude.pitch = MPU6050.KalmanAngleY;
    current_attitude.yaw = MPU6050.Gz;  // 使用陀螺仪Z轴角速度作为偏航参考
    
    // 根据飞行模式执行不同的控制策略
    switch (mode) {
        case MODE_STANDBY:
            // 待机模式 - 保持最小转速和中立位置
            control_output.left_wing = 1500;  // 中立位置
            control_output.right_wing = 1500; // 中立位置
            control_output.thruster_1 = 1000; // 最小输出
            control_output.thruster_2 = 1000; // 最小输出
            break;
            
        case MODE_SEARCH:
            // 搜索模式 - 执行预定义的搜索模式
            // 可以实现简单的扫描动作
            simulate_target(&current_target, &current_attitude);
            search_pattern(&current_attitude, &control_output, dt);
            break;
            
        case MODE_TRACKING:
            // 追踪模式 - 获取目标信息并追踪
            simulate_target(&current_target, &current_attitude); // 实际系统中应从视觉系统获取目标信息
            
            if (current_target.detected) {
                // 目标追踪控制
                target_tracking(&current_attitude, &current_target, &control_output);
            } else {
                // 没有检测到目标，切换到搜索模式逻辑
                search_pattern(&current_attitude, &control_output, dt);
            }
            break;
            
        case MODE_ATTACK:
            // 攻击模式 - 接近目标并准备攻击
            simulate_target(&current_target, &current_attitude);
            
            if (current_target.detected) {
                // 如果检测到目标，进入攻击姿态
                attack_control(&current_attitude, &current_target, &control_output);
            } else {
                // 未检测到目标，恢复到追踪模式逻辑
                target_tracking(&current_attitude, &current_target, &control_output);
            }
            break;
            
        case MODE_RETURN:
            // 返回模式 - 返回预定义的位置
            return_to_base(&current_attitude, &control_output, dt);
            break;
            
        case MODE_FAILSAFE:
            // 故障保护模式 - 执行安全着陆或其他安全操作
            failsafe_control(&current_attitude, &control_output);
            break;
            
        default:
            // 默认回到待机模式
            control_output.left_wing = 1500;
            control_output.right_wing = 1500;
            control_output.thruster_1 = 1000;
            control_output.thruster_2 = 1000;
            break;
    }
    
    // 输出控制信号到执行器
    pwm_set_servo(SERVO_LEFT_WING, control_output.left_wing);
    pwm_set_servo(SERVO_RIGHT_WING, control_output.right_wing);
    pwm_set_motor(MOTOR_THRUSTER_1, control_output.thruster_1);
    pwm_set_motor(MOTOR_THRUSTER_2, control_output.thruster_2);
    
    // 更新飞行状态机
    flight_mode_update(&current_attitude, &current_target, &control_output, dt);
}

/**
 * @brief  搜索模式下的运动模式
 * @param  attitude: 当前姿态
 * @param  control: 控制输出
 * @param  dt: 时间增量
 * @retval None
 */
static void search_pattern(const Attitude_t *attitude, Control_t *control, float dt)
{
    static float search_timer = 0;
    static int search_phase = 0;
    
    search_timer += dt;
    
    // 简单的搜索模式 - 左右扫描
    if (search_phase == 0) {
        // 向左扫描
        control->left_wing = 1400;  // 轻微左转
        control->right_wing = 1600;
        control->thruster_1 = 1300; // 中低速
        control->thruster_2 = 1300;
        
        if (search_timer > 2.0f) {
            search_timer = 0;
            search_phase = 1;
        }
    } else {
        // 向右扫描
        control->left_wing = 1600;  // 轻微右转
        control->right_wing = 1400;
        control->thruster_1 = 1300;
        control->thruster_2 = 1300;
        
        if (search_timer > 2.0f) {
            search_timer = 0;
            search_phase = 0;
        }
    }
}

/**
 * @brief  攻击模式下的控制逻辑
 * @param  attitude: 当前姿态
 * @param  target: 目标信息
 * @param  control: 控制输出
 * @retval None
 */
static void attack_control(const Attitude_t *attitude, const Target_t *target, Control_t *control)
{
    // 攻击模式下增加推力并保持目标对准
    target_tracking(attitude, target, control);
    
    // 在攻击模式下增加推进器功率
    control->thruster_1 = 1800;  // 高速模式
    control->thruster_2 = 1800;
}

/**
 * @brief  返回基地的控制逻辑
 * @param  attitude: 当前姿态
 * @param  control: 控制输出
 * @param  dt: 时间增量
 * @retval None
 */
static void return_to_base(const Attitude_t *attitude, Control_t *control, float dt)
{
    // 简单实现：保持水平并减速
    control->left_wing = 1500;   // 中立位置
    control->right_wing = 1500;
    control->thruster_1 = 1200;  // 低速飞行
    control->thruster_2 = 1200;
}

/**
 * @brief  故障保护模式下的控制逻辑
 * @param  attitude: 当前姿态
 * @param  control: 控制输出
 * @retval None
 */
static void failsafe_control(const Attitude_t *attitude, Control_t *control)
{
    // 故障保护：尝试保持水平姿态，减小推力
    control->left_wing = 1500;   // 中立位置
    control->right_wing = 1500;
    
    // 缓慢减小推力以降低速度
    if (control->thruster_1 > 1100) {
        control->thruster_1 -= 10;
    }
    if (control->thruster_2 > 1100) {
        control->thruster_2 -= 10;
    }
}
