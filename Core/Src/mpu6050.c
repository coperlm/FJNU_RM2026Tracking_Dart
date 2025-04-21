#include "mpu6050.h"
#include "i2c.h"
#include <math.h>

/* 陀螺仪和加速度计转换系数 */
#define GYRO_FACTOR_250DPS      (250.0f / 32768.0f)
#define GYRO_FACTOR_500DPS      (500.0f / 32768.0f)
#define GYRO_FACTOR_1000DPS     (1000.0f / 32768.0f)
#define GYRO_FACTOR_2000DPS     (2000.0f / 32768.0f)

#define ACCEL_FACTOR_2G         (2.0f / 32768.0f)
#define ACCEL_FACTOR_4G         (4.0f / 32768.0f)
#define ACCEL_FACTOR_8G         (8.0f / 32768.0f)
#define ACCEL_FACTOR_16G        (16.0f / 32768.0f)

/* 静态全局变量 */
static MPU6050_Data_t mpu_data;
static float gyro_scale_factor;
static float accel_scale_factor;

/**
 * @brief 初始化MPU6050
 * 
 * @return HAL_StatusTypeDef 初始化状态
 */
HAL_StatusTypeDef MPU6050_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t check, data;
    
    // 检查设备ID
    status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &check, 1, 100);
    if (status != HAL_OK || check != 0x68) {
        return HAL_ERROR;  // 设备ID不匹配或通信失败
    }
    
    // 复位设备
    data = 0x80;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &data, 1, 100);
    if (status != HAL_OK) return status;
    HAL_Delay(100);  // 等待复位完成
    
    // 唤醒MPU6050
    data = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &data, 1, 100);
    if (status != HAL_OK) return status;
    
    // 配置采样率 = 陀螺仪输出率 / (1 + SMPLRT_DIV)
    data = 1000 / MPU_SAMPLE_RATE - 1;  // 陀螺仪输出率默认为1kHz
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_SMPLRT_DIV, 1, &data, 1, 100);
    if (status != HAL_OK) return status;
    
    // 配置数字低通滤波器
    data = 0x03;  // 配置DLPF，带宽为42Hz
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_CONFIG, 1, &data, 1, 100);
    if (status != HAL_OK) return status;
    
    // 配置陀螺仪量程
    data = GYRO_SCALE;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &data, 1, 100);
    if (status != HAL_OK) return status;
    
    // 根据设置的量程选择转换因子
    switch (GYRO_SCALE) {
        case GYRO_SCALE_250DPS:  gyro_scale_factor = GYRO_FACTOR_250DPS;  break;
        case GYRO_SCALE_500DPS:  gyro_scale_factor = GYRO_FACTOR_500DPS;  break;
        case GYRO_SCALE_1000DPS: gyro_scale_factor = GYRO_FACTOR_1000DPS; break;
        case GYRO_SCALE_2000DPS: gyro_scale_factor = GYRO_FACTOR_2000DPS; break;
        default: gyro_scale_factor = GYRO_FACTOR_1000DPS;
    }
    
    // 配置加速度计量程
    data = ACCEL_SCALE;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &data, 1, 100);
    if (status != HAL_OK) return status;
    
    // 根据设置的量程选择转换因子
    switch (ACCEL_SCALE) {
        case ACCEL_SCALE_2G:  accel_scale_factor = ACCEL_FACTOR_2G;  break;
        case ACCEL_SCALE_4G:  accel_scale_factor = ACCEL_FACTOR_4G;  break;
        case ACCEL_SCALE_8G:  accel_scale_factor = ACCEL_FACTOR_8G;  break;
        case ACCEL_SCALE_16G: accel_scale_factor = ACCEL_FACTOR_16G; break;
        default: accel_scale_factor = ACCEL_FACTOR_8G;
    }
    
    return HAL_OK;
}

/**
 * @brief 读取MPU6050传感器数据
 * 
 * @param data 输出传感器数据指针
 * @return HAL_StatusTypeDef 读取状态
 */
HAL_StatusTypeDef MPU6050_ReadSensor(MPU6050_Data_t *data)
{
    uint8_t buffer[14];
    int16_t raw_accel_x, raw_accel_y, raw_accel_z;
    int16_t raw_temp;
    int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
    HAL_StatusTypeDef status;
    
    // 从0x3B读取14个字节的数据 (加速度、温度和角速度)
    status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, buffer, 14, 100);
    if (status != HAL_OK) return status;
    
    // 合并高字节和低字节
    raw_accel_x = (int16_t)(buffer[0] << 8 | buffer[1]);
    raw_accel_y = (int16_t)(buffer[2] << 8 | buffer[3]);
    raw_accel_z = (int16_t)(buffer[4] << 8 | buffer[5]);
    raw_temp    = (int16_t)(buffer[6] << 8 | buffer[7]);
    raw_gyro_x  = (int16_t)(buffer[8] << 8 | buffer[9]);
    raw_gyro_y  = (int16_t)(buffer[10] << 8 | buffer[11]);
    raw_gyro_z  = (int16_t)(buffer[12] << 8 | buffer[13]);
    
    // 转换为实际物理值
    data->accel_x = raw_accel_x * accel_scale_factor;
    data->accel_y = raw_accel_y * accel_scale_factor;
    data->accel_z = raw_accel_z * accel_scale_factor;
    data->temp = (raw_temp / 340.0f) + 36.53f;  // 温度转换公式来自MPU6050数据手册
    data->gyro_x = raw_gyro_x * gyro_scale_factor;
    data->gyro_y = raw_gyro_y * gyro_scale_factor;
    data->gyro_z = raw_gyro_z * gyro_scale_factor;
    
    // 同时更新全局数据
    mpu_data = *data;
    
    return HAL_OK;
}

/**
 * @brief 从加速度计数据计算倾角
 * 
 * @param mpu_data MPU6050数据指针
 * @param roll 输出横滚角指针
 * @param pitch 输出俯仰角指针
 */
void calculate_angles_from_accel(const MPU6050_Data_t *mpu_data, float *roll, float *pitch)
{
    // 从加速度计数据计算倾角
    *roll = atan2f(mpu_data->accel_y, mpu_data->accel_z) * 57.3f; // 转换为角度
    *pitch = atan2f(-mpu_data->accel_x, sqrtf(mpu_data->accel_y * mpu_data->accel_y + 
                                           mpu_data->accel_z * mpu_data->accel_z)) * 57.3f;
}

/**
 * @brief 获取最新MPU6050数据
 * 
 * @return MPU6050_Data_t* MPU数据指针
 */
MPU6050_Data_t* MPU6050_GetData(void)
{
    return &mpu_data;
}