#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "mpu6050.h"
#include "dart_control.h"

/* 卡尔曼滤波参数 */
#define KF_Q_ANGLE              0.001f  // 角度过程噪声方差
#define KF_Q_BIAS               0.003f  // 角速度偏差过程噪声方差
#define KF_R_MEASURE            0.03f   // 测量噪声方差

/* 互补滤波系数 */
#define COMP_FILTER_ALPHA       0.96f

/* 卡尔曼滤波器结构 */
typedef struct {
    float Q_angle;   // 过程噪声协方差
    float Q_bias;    // 过程噪声协方差
    float R_measure; // 测量噪声协方差
    float angle;     // 角度
    float bias;      // 角速度偏差
    float P[2][2];   // 误差协方差矩阵
} Kalman_t;

/* 函数声明 */
void Kalman_Init(Kalman_t *kalman);
float Kalman_Update(Kalman_t *kalman, float newAngle, float newRate, float dt);
void read_mpu6050_attitude(Attitude_t *attitude);

#endif /* KALMAN_FILTER_H */