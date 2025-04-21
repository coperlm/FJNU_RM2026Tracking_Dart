#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include "dart_control.h"

/**
 * @brief 初始化PWM输出
 * 已适配STM32 HAL库
 */
void pwm_init(void);

/**
 * @brief 设置舵机PWM值
 * 
 * @param channel 舵机通道
 * @param pulse_width_us PWM脉宽(微秒)
 */
void pwm_set_servo(uint8_t channel, uint16_t pulse_width_us);

/**
 * @brief 设置电机PWM值
 * 
 * @param channel 电机通道
 * @param pulse_width_us PWM脉宽(微秒)
 */
void pwm_set_motor(uint8_t channel, uint16_t pulse_width_us);

#endif /* PWM_CONTROL_H */