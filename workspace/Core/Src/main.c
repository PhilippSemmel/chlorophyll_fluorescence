/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "helper_functions.h"
#include "AS7265x.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

struct AS7265_dev spectrometer;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
int8_t A7265x_Read(uint8_t devAddr, uint8_t reg, uint8_t *data, uint16_t size);
int8_t A7265x_Read(uint8_t devAddr, uint8_t reg, uint8_t *data, uint16_t size);
int8_t I2C_ReadRegister(uint8_t devAddr, uint8_t reg, uint8_t *data, uint16_t size);
int8_t I2C_WriteRegister(uint8_t devAddr, uint8_t reg, uint8_t *data, uint16_t size);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  spectrometer.devAddr = AS7265_I2C_ADDR;
  spectrometer.read = I2C_ReadRegister;
  spectrometer.write = I2C_WriteRegister;

//  printf("Test\r\n");
//  uint8_t d = 0;
//  spectrometer.read(spectrometer.devAddr, AS7265_I2C_SLAVE_STATUS_REG, &d,
//      1);
//  printf("STATUS Register: 0x%x\r\n", d);
//  spectrometer.read(spectrometer.devAddr, AS7265_I2C_SLAVE_WRITE_REG, &d, 1);
//  printf("Write Register: 0x%x\r\n", d);
//  spectrometer.read(spectrometer.devAddr, AS7265_I2C_SLAVE_READ_REG, &d, 1);
//  printf("Read Register: 0x%x\r\n", d);
//  printf("Virtual Reg\r\n");
//  uint8_t data_h = 0;
//  uint8_t data_l = 0;
//  AS7265_read_virtual_reg(&spectrometer, AS7265_RAW_VALUE_0_H, &data_h);
//  AS7265_read_virtual_reg(&spectrometer, AS7265_RAW_VALUE_0_L, &data_l);
//  printf("raw spectrometer data: 0x%x%x\r\n", data_h, data_l);
//  printf("direct read\r\n");
//  spectrometer.read(spectrometer.devAddr, AS7265_RAW_VALUE_0_H, &data_h, 1);
//  spectrometer.read(spectrometer.devAddr, AS7265_RAW_VALUE_0_L, &data_l, 1);
//  printf("raw spectrometer data: 0x%x%x\r\n", data_h, data_l);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int count = 0;
  uint8_t status;
  uint8_t data;
  uint8_t virtualRegAddr = AS7265_RAW_VALUE_0_L;
  while (1) {
    printf("count: %i\r\n", count);
    while (1) {
      // Read slave I²C status to see if the read buffer is ready.
      spectrometer.read(spectrometer.devAddr, AS7265_I2C_SLAVE_STATUS_REG,
              &status, 1);
      if ((status & AS7265_I2C_SLAVE_TX_VALID) == 0) {
        // No inbound TX pending at slave. Okay to write now.
        printf("TX valid: 0x%x\r\n", status);
        break;
      }
      printf("TX not valid: 0x%x\r\n", status);
    }

    // Send the virtual register address (disabling bit 7 to indicate a read).
    printf("virtual reg addr 0x%x\r\n", virtualRegAddr);
    spectrometer.write(spectrometer.devAddr, AS7265_I2C_SLAVE_WRITE_REG,
        &virtualRegAddr, 1);

    while (1) {
      spectrometer.read(spectrometer.devAddr, AS7265_I2C_SLAVE_STATUS_REG,
                    &status, 1);
      if ((status & AS7265_I2C_SLAVE_RX_VALID) != 0) {
        // Read data is ready.
        printf("RX valid: 0x%x\r\n", status);
        break;
      }
      printf("RX not valid: 0x%x\r\n", status);
    }
    // Read the data to complete the operation.
    spectrometer.read(spectrometer.devAddr, AS7265_I2C_SLAVE_READ_REG,
                  &data, 1);
    printf("readings: 0x%x\r\n", data);
    HAL_Delay(1000);
    count++;
//    return data;
//    printf("wait for tx to be valid\r\n");
//    while (1) {
//      spectrometer.read(spectrometer.devAddr, AS7265_I2C_SLAVE_STATUS_REG, &data, 1);
//      if (data != 0x80) {
//        HAL_Delay(50);
//        printf("TX not valid: 0x%x\r\n", data);
//      } else {
//        break;
//      }
//    }
//    printf("TX valid: 0x%x\r\n", data);
//
//    uint8_t reg = AS7265_RAW_VALUE_0_H;
//    spectrometer.write(spectrometer.devAddr, AS7265_I2C_SLAVE_WRITE_REG, &reg, 1);
//
//    printf("wait for rx to be valid\r\n");
//    while (1) {
//      spectrometer.read(spectrometer.devAddr, AS7265_I2C_SLAVE_STATUS_REG, &data, 1);
//      if (data != 0x40) {
//        HAL_Delay(50);
//        printf("RX not valid: 0x%x\r\n", data);
//      } else {
//        break;
//      }
//    }
//    printf("RX valid: 0x%x\r\n", data);
//
//    spectrometer.read(spectrometer.devAddr, AS7265_I2C_SLAVE_READ_REG, &data, 1);
//    printf("readings: 0x%x\r\n", data);
//    HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int8_t A7265x_Read(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t size) {
  HAL_StatusTypeDef status;
  uint16_t write_addr = (dev_addr << 1) | AS7265_I2C_SLAVE_WRITE_REG;
  uint16_t read_addr = (dev_addr << 1) | AS7265_I2C_SLAVE_READ_REG;
  status = HAL_I2C_Master_Transmit(&hi2c1, write_addr, &reg, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    return AS7265_I2C_COMM_ERROR;
  }

  // Then, read the data from the module
  status = HAL_I2C_Master_Receive(&hi2c1, read_addr, data, size, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    return AS7265_I2C_COMM_ERROR;
  }
  return AS7265_I2C_SUCCESS;
}

int8_t I2C_ReadRegister(uint8_t devAddr, uint8_t reg, uint8_t *data,
    uint16_t size) {
  HAL_StatusTypeDef status;
  uint8_t readAddr = (devAddr << 1) | 0x01;
  status = HAL_I2C_Mem_Read(&hi2c1, readAddr, reg, I2C_MEMADD_SIZE_8BIT, data,
      size, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    return AS7265_I2C_COMM_ERROR;
  }
  return AS7265_I2C_SUCCESS;
}

int8_t I2C_WriteRegister(uint8_t devAddr, uint8_t reg, uint8_t *data,
    uint16_t size) {
  HAL_StatusTypeDef status;
  uint8_t writeAddr = (devAddr << 1) | 0x00;
  status = HAL_I2C_Mem_Write(&hi2c1, writeAddr, reg, I2C_MEMADD_SIZE_8BIT, data,
      size, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    return AS7265_I2C_COMM_ERROR;
  }
  return AS7265_I2C_SUCCESS;
}

int8_t A7265x_Write(uint8_t devAddr, uint8_t reg, uint8_t *data, uint16_t size) {
  HAL_StatusTypeDef status;
  uint16_t write_addr = (devAddr << 1) | AS7265_I2C_SLAVE_WRITE_REG;
  status = HAL_I2C_Master_Transmit(&hi2c1, write_addr, data, 1,
  HAL_MAX_DELAY);
  if (status != HAL_OK) {
    return AS7265_I2C_COMM_ERROR;
  }
  return AS7265_I2C_SUCCESS;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
