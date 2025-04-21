#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include <stdbool.h>

/* MPU6050寄存器地址定义 */
#define MPU6050_ADDR            0xD0 // MPU6050 I2C地址 (0x68 << 1)
#define MPU6050_SMPLRT_DIV      0x19 // 采样率分频寄存器
#define MPU6050_CONFIG          0x1A // 配置寄存器
#define MPU6050_GYRO_CONFIG     0x1B // 陀螺仪配置寄存器
#define MPU6050_ACCEL_CONFIG    0x1C // 加速度计配置寄存器
#define MPU6050_FIFO_EN         0x23 // FIFO使能寄存器
#define MPU6050_INT_ENABLE      0x38 // 中断使能寄存器
#define MPU6050_ACCEL_XOUT_H    0x3B // 加速度X轴高字节
#define MPU6050_ACCEL_XOUT_L    0x3C // 加速度X轴低字节
#define MPU6050_ACCEL_YOUT_H    0x3D // 加速度Y轴高字节
#define MPU6050_ACCEL_YOUT_L    0x3E // 加速度Y轴低字节
#define MPU6050_ACCEL_ZOUT_H    0x3F // 加速度Z轴高字节
#define MPU6050_ACCEL_ZOUT_L    0x40 // 加速度Z轴低字节
#define MPU6050_TEMP_OUT_H      0x41 // 温度高字节
#define MPU6050_TEMP_OUT_L      0x42 // 温度低字节
#define MPU6050_GYRO_XOUT_H     0x43 // 陀螺仪X轴高字节
#define MPU6050_GYRO_XOUT_L     0x44 // 陀螺仪X轴低字节
#define MPU6050_GYRO_YOUT_H     0x45 // 陀螺仪Y轴高字节
#define MPU6050_GYRO_YOUT_L     0x46 // 陀螺仪Y轴低字节
#define MPU6050_GYRO_ZOUT_H     0x47 // 陀螺仪Z轴高字节
#define MPU6050_GYRO_ZOUT_L     0x48 // 陀螺仪Z轴低字节
#define MPU6050_PWR_MGMT_1      0x6B // 电源管理1寄存器
#define MPU6050_PWR_MGMT_2      0x6C // 电源管理2寄存器
#define MPU6050_WHO_AM_I        0x75 // 设备ID寄存器

/* 陀螺仪和加速度计量程配置 */
#define GYRO_SCALE_250DPS       0x00 // ±250°/s
#define GYRO_SCALE_500DPS       0x08 // ±500°/s
#define GYRO_SCALE_1000DPS      0x10 // ±1000°/s
#define GYRO_SCALE_2000DPS      0x18 // ±2000°/s

#define ACCEL_SCALE_2G          0x00 // ±2g
#define ACCEL_SCALE_4G          0x08 // ±4g
#define ACCEL_SCALE_8G          0x10 // ±8g
#define ACCEL_SCALE_16G         0x18 // ±16g

/* 配置参数 */
#define GYRO_SCALE              GYRO_SCALE_1000DPS
#define ACCEL_SCALE             ACCEL_SCALE_8G
#define MPU_SAMPLE_RATE         100 // Hz
#define DT                      0.01 // 采样周期, 秒

/* 传感器数据结构 */
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
} MPU6050_Data_t;

/* 函数声明 */
HAL_StatusTypeDef MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_ReadSensor(MPU6050_Data_t *data);
void calculate_angles_from_accel(const MPU6050_Data_t *mpu_data, float *roll, float *pitch);
MPU6050_Data_t* MPU6050_GetData(void);

#endif /* MPU6050_H */