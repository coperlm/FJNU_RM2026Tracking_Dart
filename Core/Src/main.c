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
#include "oled.h"  // Include OLED header file
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOOP_INTERVAL_MS 10 // Control loop interval 10ms (100Hz)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Attitude_t attitude;  // Attitude data
Target_t target;      // Target data
Control_t control;    // Control output data
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void delay_ms(uint16_t ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Redirect printf to UART1 and OLED display
 * @param ch Character
 * @param f File pointer (not used)
 * @return int Character written
 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  // Send character through UART1
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

  // Also send to OLED screen for display
  static char line_buffer[128]; // OLED line buffer
  static uint8_t buffer_index = 0; // Buffer index

  // Store character to buffer (ignore carriage return '\r')
  if (ch != '\r') {
    if (buffer_index < sizeof(line_buffer) - 1) {
      line_buffer[buffer_index++] = ch;
    }
  }

  // On newline '\n' or full buffer, display the line on OLED
  if (ch == '\n' || buffer_index >= sizeof(line_buffer) - 1) {
    line_buffer[buffer_index] = '\0';  // Add string terminator
    OLED_Printf("%s", line_buffer);    // Print buffer content on OLED
    buffer_index = 0;                  // Reset buffer index
  }

  return ch;
}

/**
 * @brief Millisecond delay function (based on HAL_Delay)
 * @param ms Number of milliseconds to delay
 */
void delay_ms(uint16_t ms)
{
  HAL_Delay(ms);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // User code can be added here - before initialization
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // User code can be added here - before system clock configuration
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // User code can be added here - before peripheral initialization
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize random number generator - using system tick as seed
  srand(HAL_GetTick());

  // Initialize OLED display
  OLED_Init();
  HAL_Delay(100);  // Wait for OLED stabilization
  OLED_Clear();    // Clear screen

  // Output startup information through printf (redirected to UART and OLED)
  printf("\n\rDart Flight Control System Start...\n\r");

  // Initialize dart control core system (including attitude, PID, etc.)
  dart_system_init();

  // Output initialization complete message
  printf("System Init Done. Starting Control Loop...\n\r\n\r");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Main control loop
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 1. Get attitude data (currently using simulated data)
    //    In actual application, replace with MPU6050 or other IMU sensor data reading
    simulate_gyro(&attitude);

    // 2. Get target data (currently using simulated data)
    //    In actual application, replace with vision module or other target detection system input
    simulate_target(&target, &attitude);

    // 3. Update attitude control logic (calculate servo/motor output)
    update_attitude_control(&attitude, &target, &control);

    // 4. Output debug information to UART and OLED
    //    Display current attitude (roll, pitch, yaw)
    printf("Att: Roll=%.1f, Pitch=%.1f, Yaw=%.1f | ", attitude.roll, attitude.pitch, attitude.yaw);

    //    Display target detection status and coordinates
    if (target.detected) {
        printf("Target: X=%.2f, Y=%.2f [Detected]\n\r", target.x, target.y);
    } else {
        printf("Target: Not Detected\n\r");
    }

    // 5. Control loop frequency (delay to achieve approximately 100Hz control frequency)
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
