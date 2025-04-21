#include "dart_control.h"
#include "tim.h"
#include <stdio.h>

/**
 * @brief 初始化PWM输出
 * 已适配STM32 HAL库
 */
void pwm_init(void) {
    // 启动定时器的PWM输出
    HAL_TIM_PWM_Start(&htim1, SERVO_LEFT_CHANNEL);  // 左舵机
    HAL_TIM_PWM_Start(&htim1, SERVO_RIGHT_CHANNEL); // 右舵机
    HAL_TIM_PWM_Start(&htim2, MOTOR_CHANNEL);       // 电机
    
    // 设置舵机初始位置为中间位置
    pwm_set_servo(SERVO_LEFT_WING, 1500);
    pwm_set_servo(SERVO_RIGHT_WING, 1500);
    
    // 设置电机初始值为最低速度
    pwm_set_motor(MOTOR_THRUSTER, MOTOR_MIN_PULSE);
}

/**
 * @brief 设置舵机PWM值
 * 
 * @param channel 舵机通道
 * @param pulse_width_us PWM脉宽(微秒)
 */
void pwm_set_servo(uint8_t channel, uint16_t pulse_width_us) {
    // 限制脉宽在有效范围内
    if (pulse_width_us < SERVO_MIN_PULSE) {
        pulse_width_us = SERVO_MIN_PULSE;
    } else if (pulse_width_us > SERVO_MAX_PULSE) {
        pulse_width_us = SERVO_MAX_PULSE;
    }
    
    // 计算PWM占空比值
    // 对于50Hz的PWM，周期是20ms(20000us)
    // pulse_width_us / 20000 = pulse_width / ARR
    // 因此 pulse_width = (pulse_width_us * ARR) / 20000
    uint32_t period = htim1.Instance->ARR;
    uint32_t pulse = (pulse_width_us * period) / 20000;

    // 根据不同的通道设置PWM值
    if (channel == SERVO_LEFT_WING) {
        __HAL_TIM_SET_COMPARE(&htim1, SERVO_LEFT_CHANNEL, pulse);
    } else if (channel == SERVO_RIGHT_WING) {
        __HAL_TIM_SET_COMPARE(&htim1, SERVO_RIGHT_CHANNEL, pulse);
    }
}

/**
 * @brief 设置电机PWM值
 * 
 * @param channel 电机通道
 * @param pulse_width_us PWM脉宽(微秒)
 */
void pwm_set_motor(uint8_t channel, uint16_t pulse_width_us) {
    // 限制脉宽在有效范围内
    if (pulse_width_us < MOTOR_MIN_PULSE) {
        pulse_width_us = MOTOR_MIN_PULSE;
    } else if (pulse_width_us > MOTOR_MAX_PULSE) {
        pulse_width_us = MOTOR_MAX_PULSE;
    }
    
    // 计算PWM占空比值
    uint32_t period = htim2.Instance->ARR;
    uint32_t pulse = (pulse_width_us * period) / 20000;

    // 设置PWM值
    if (channel == MOTOR_THRUSTER) {
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR_CHANNEL, pulse);
    }
}
