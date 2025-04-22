/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dart_control.h"
#include "oled.h"  // 添加OLED头文件
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOOP_INTERVAL_MS 10 // 控制循环间隔10ms (100Hz)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Attitude_t attitude;  // 姿态数据
Target_t target;      // 目标数据
Control_t control;    // 控制输出数据
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void delay_ms(uint16_t ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief 重定向printf到UART1和OLED显示器
 * @param ch 字符
 * @param f 文件指针 (未使用)
 * @return int 写入的字符
 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  // 通过 UART1 发送字符
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

  // 同时发送到OLED屏幕进行显示
  static char line_buffer[128]; // OLED行缓冲区
  static uint8_t buffer_index = 0; // 缓冲区索引

  // 存储字符到缓冲区 (忽略回车符 '\r')
  if (ch != '\r') {
    if (buffer_index < sizeof(line_buffer) - 1) {
      line_buffer[buffer_index++] = ch;
    }
  }

  // 遇到换行符 '\n' 或缓冲区满时，在OLED上显示该行
  if (ch == '\n' || buffer_index >= sizeof(line_buffer) - 1) {
    line_buffer[buffer_index] = '\0';  // 添加字符串结束符
    OLED_Printf("%s", line_buffer);   // 在OLED上打印缓冲区内容
    buffer_index = 0;                 // 重置缓冲区索引
  }

  return ch;
}

/**
 * @brief 毫秒级延时函数 (基于 HAL_Delay)
 * @param ms 要延时的毫秒数
 */
void delay_ms(uint16_t ms)
{
  HAL_Delay(ms);
}
/* USER CODE END 0 */

/**
  * @brief  应用程序入口点.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // 此处可添加用户代码 - 初始化前
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* 复位所有外设, 初始化 Flash 接口和 Systick */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // 此处可添加用户代码 - 系统时钟配置前
  /* USER CODE END Init */

  /* 配置系统时钟 */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // 此处可添加用户代码 - 外设初始化前
  /* USER CODE END SysInit */

  /* 初始化所有已配置的外设 */
  MX_GPIO_Init();     // 初始化 GPIO
  MX_I2C1_Init();     // 初始化 I2C1 (可能用于传感器)
  MX_I2C2_Init();     // 初始化 I2C2 (可能用于OLED)
  MX_TIM1_Init();     // 初始化 TIM1 (可能用于PWM输出 - 舵机/电机)
  MX_TIM2_Init();     // 初始化 TIM2 (可能用于PWM输出或定时)
  MX_USART1_UART_Init(); // 初始化 USART1 (用于调试信息输出)
  /* USER CODE BEGIN 2 */

  // 初始化随机数生成器 - 使用系统滴答定时器值作为种子
  srand(HAL_GetTick());

  // 初始化OLED显示屏
  OLED_Init();
  HAL_Delay(100);  // 等待OLED稳定
  OLED_Clear();    // 清屏

  // 通过 printf (已重定向到 UART 和 OLED) 输出启动信息
  printf("\n\rDart Flight Control System Start...\n\r"); // 飞镖飞控系统启动...


  // 初始化飞镖控制核心系统 (包括姿态、PID等)
  dart_system_init();

  // 输出初始化完成信息
  printf("System Init Done. Starting Control Loop...\n\r\n\r"); // 系统初始化完成，开始控制循环...

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // 主控制循环
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 1. 获取姿态数据 (当前使用模拟数据)
    //    实际应用中应替换为 MPU6050 或其他 IMU 传感器的数据读取和解算
    simulate_gyro(&attitude);

    // 2. 获取目标数据 (当前使用模拟数据)
    //    实际应用中应替换为视觉模块或其他目标探测系统的数据输入
    simulate_target(&target, &attitude);

    // 3. 更新姿态控制逻辑 (计算舵机/电机输出)
    update_attitude_control(&attitude, &target, &control);

    // 4. 输出调试信息到 UART 和 OLED
    //    显示当前姿态 (横滚、俯仰、偏航)
    printf("Att: Roll=%.1f, Pitch=%.1f, Yaw=%.1f | ", attitude.roll, attitude.pitch, attitude.yaw);

    //    显示目标检测状态和坐标
    if (target.detected) {
        printf("Target: X=%.2f, Y=%.2f [Detected]\n\r", target.x, target.y); // 目标: X=..., Y=... [检测到]
    } else {
        printf("Target: Not Detected\n\r"); // 目标: 未检测到
    }

    // 5. 控制循环频率 (延时以达到约 100Hz 的控制频率)
    delay_ms(LOOP_INTERVAL_MS);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
