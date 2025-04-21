#include "dart_control.h"
#include <math.h>
#include <stdlib.h>

/**
 * @brief 模拟陀螺仪数据
 * 在实际应用中，应替换为实际的IMU传感器读取
 * 
 * @param attitude 输出的姿态数据结构体指针
 */
void simulate_gyro(Attitude_t *attitude) {
    static float time_counter = 0.0f;
    
    // 模拟飞行过程中的姿态变化
    time_counter += 0.01f;
    
    // 生成一些随机扰动，模拟实际飞行环境下的姿态变化
    attitude->roll = 5.0f * sinf(time_counter) + ((float)rand() / RAND_MAX - 0.5f) * 2.0f;
    attitude->pitch = 3.0f * cosf(time_counter * 0.7f) + ((float)rand() / RAND_MAX - 0.5f) * 2.0f;
    attitude->yaw = 10.0f * sinf(time_counter * 0.3f) + ((float)rand() / RAND_MAX - 0.5f) * 5.0f;
}

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
