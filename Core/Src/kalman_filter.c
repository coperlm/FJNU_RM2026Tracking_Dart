#include "kalman_filter.h"
#include <math.h>

/* 静态全局变量 */
static Kalman_t kalman_roll, kalman_pitch;
static float roll_comp = 0.0f;   // 横滚角 (互补滤波)
static float pitch_comp = 0.0f;  // 俯仰角 (互补滤波)
static float yaw_comp = 0.0f;    // 偏航角 (互补滤波)

/* 全局导出的卡尔曼滤波器实例，供其他模块使用 */
Kalman_t KalmanX, KalmanY;

/**
 * @brief 初始化卡尔曼滤波器
 * 
 * @param kalman 卡尔曼滤波器结构体指针
 */
void Kalman_Init(Kalman_t *kalman)
{
    kalman->Q_angle = KF_Q_ANGLE;
    kalman->Q_bias = KF_Q_BIAS;
    kalman->R_measure = KF_R_MEASURE;
    
    kalman->angle = 0.0f;
    kalman->bias = 0.0f;
    
    kalman->P[0][0] = 0.0f;
    kalman->P[0][1] = 0.0f;
    kalman->P[1][0] = 0.0f;
    kalman->P[1][1] = 0.0f;
}

/**
 * @brief 卡尔曼滤波器更新
 * 
 * @param kalman 卡尔曼滤波器结构体指针
 * @param newAngle 从加速度计获取的角度
 * @param newRate 从陀螺仪获取的角速度
 * @param dt 时间间隔
 * @return float 滤波后的角度
 */
float Kalman_Update(Kalman_t *kalman, float newAngle, float newRate, float dt)
{
    // 步骤1: 预测
    // 基于上一状态的角度预测
    kalman->angle += dt * (newRate - kalman->bias);
    
    // 更新误差协方差矩阵
    kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;
    
    // 步骤2: 校正
    // 计算卡尔曼增益
    float S = kalman->P[0][0] + kalman->R_measure;
    float K[2];
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;
    
    // 使用测量值更新估计
    float y = newAngle - kalman->angle;
    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;
    
    // 更新误差协方差矩阵
    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];
    
    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;
    
    return kalman->angle;
}

/**
 * @brief 从MPU6050读取姿态数据并应用卡尔曼滤波
 * 
 * @param attitude 输出的姿态数据结构体指针
 */
void read_mpu6050_attitude(Attitude_t *attitude)
{
    static uint32_t last_time = 0;
    uint32_t current_time;
    float dt;
    float accel_roll, accel_pitch;
    static bool is_first_run = true;
    MPU6050_Data_t mpu_data_local;
    
    // 首次运行时初始化卡尔曼滤波器
    if (is_first_run) {
        Kalman_Init(&kalman_roll);
        Kalman_Init(&kalman_pitch);
        
        // 初始化全局卡尔曼滤波器实例
        Kalman_Init(&KalmanX);
        Kalman_Init(&KalmanY);
        
        is_first_run = false;
        last_time = HAL_GetTick();
    }
    
    // 计算时间间隔 (秒)
    current_time = HAL_GetTick();
    dt = (current_time - last_time) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f; // 防止dt太小
    last_time = current_time;
    
    // 读取MPU6050数据
    if (MPU6050_ReadSensor(&mpu_data_local) != HAL_OK) {
        return; // 读取失败则保持上一次的姿态数据
    }
    
    // 从加速度计计算倾角
    calculate_angles_from_accel(&mpu_data_local, &accel_roll, &accel_pitch);
    
    // 卡尔曼滤波器更新
    attitude->roll = Kalman_Update(&kalman_roll, accel_roll, mpu_data_local.gyro_x, dt);
    attitude->pitch = Kalman_Update(&kalman_pitch, accel_pitch, mpu_data_local.gyro_y, dt);
    
    // 偏航角只能靠陀螺仪积分
    attitude->yaw += mpu_data_local.gyro_z * dt;
    
    // 保持偏航角在[0, 360)范围内
    while (attitude->yaw >= 360.0f) attitude->yaw -= 360.0f;
    while (attitude->yaw < 0.0f) attitude->yaw += 360.0f;
    
    // 互补滤波 (可以作为备选方案)
    roll_comp = COMP_FILTER_ALPHA * (roll_comp + mpu_data_local.gyro_x * dt) + (1.0f - COMP_FILTER_ALPHA) * accel_roll;
    pitch_comp = COMP_FILTER_ALPHA * (pitch_comp + mpu_data_local.gyro_y * dt) + (1.0f - COMP_FILTER_ALPHA) * accel_pitch;
    yaw_comp += mpu_data_local.gyro_z * dt;
}
