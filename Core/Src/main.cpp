/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Stepper.h"
#include "my_definitions.h"
#include "Dial.h"
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
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection"))); /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void Move_Stepper(enum Direction dir, int full_turns, int next_number);
void Stop_TIM3(TIM_TypeDef *TIMx);
void Start_TIM3(TIM_TypeDef *TIMx);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

std::StepperMotor *Stepper = new std::StepperMotor(TIM3);
std::Dial *Dial = new std::Dial();

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
   MX_ETH_Init();
   MX_USART3_UART_Init();
   MX_USB_OTG_FS_PCD_Init();
   MX_TIM3_Init();
   MX_I2C2_Init();
   /* USER CODE BEGIN 2 */

   // Timer3 startup
   HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);
   TIM3->CR1 &= ~(TIM_CR1_CEN); // Disable counter

   // Stepper
   int newnum = 90; // Next number to spin the dial to
   enum Direction dir = CW; // Direction of dial rotation.

   // Encoder
   uint8_t i2c_receive_buf[2]; // Position data is 12 bits and requires two reads.
   uint16_t encoder_angle = 0; //

   /* USER CODE END 2 */

   /* Infinite loop */
   /* USER CODE BEGIN WHILE */

   while (1) {

      // Routine to demo stepper control
      Move_Stepper(dir, 0, newnum);

      if (newnum > 0) {
         newnum -= 10;
      } else {
         newnum = 90;
         if (dir == CCW) {
            dir = CW;
         } else
            (dir = CCW);
      }

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

   /** Supply configuration update enable
    */
   HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
   /** Configure the main internal regulator output voltage
    */
   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

   while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
   }
   /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
         | RCC_OSCILLATORTYPE_HSE;
   RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
   RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
   RCC_OscInitStruct.PLL.PLLM = 1;
   RCC_OscInitStruct.PLL.PLLN = 24;
   RCC_OscInitStruct.PLL.PLLP = 2;
   RCC_OscInitStruct.PLL.PLLQ = 4;
   RCC_OscInitStruct.PLL.PLLR = 2;
   RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
   RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
   RCC_OscInitStruct.PLL.PLLFRACN = 0;
   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
      Error_Handler();
   }
   /** Initializes the CPU, AHB and APB buses clocks
    */
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
         | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
         | RCC_CLOCKTYPE_D1PCLK1;
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
   RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
   RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
   RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
   RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
      Error_Handler();
   }
}

/**
 * @brief ETH Initialization Function
 * @param None
 * @retval None
 */
static void MX_ETH_Init(void) {

   /* USER CODE BEGIN ETH_Init 0 */

   /* USER CODE END ETH_Init 0 */

   /* USER CODE BEGIN ETH_Init 1 */

   /* USER CODE END ETH_Init 1 */
   heth.Instance = ETH;
   heth.Init.MACAddr[0] = 0x00;
   heth.Init.MACAddr[1] = 0x80;
   heth.Init.MACAddr[2] = 0xE1;
   heth.Init.MACAddr[3] = 0x00;
   heth.Init.MACAddr[4] = 0x00;
   heth.Init.MACAddr[5] = 0x00;
   heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
   heth.Init.TxDesc = DMATxDscrTab;
   heth.Init.RxDesc = DMARxDscrTab;
   heth.Init.RxBuffLen = 1524;

   /* USER CODE BEGIN MACADDRESS */

   /* USER CODE END MACADDRESS */

   if (HAL_ETH_Init(&heth) != HAL_OK) {
      Error_Handler();
   }

   memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
   TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM
         | ETH_TX_PACKETS_FEATURES_CRCPAD;
   TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
   TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
   /* USER CODE BEGIN ETH_Init 2 */

   /* USER CODE END ETH_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

   /* USER CODE BEGIN I2C2_Init 0 */

   /* USER CODE END I2C2_Init 0 */

   /* USER CODE BEGIN I2C2_Init 1 */

   /* USER CODE END I2C2_Init 1 */
   hi2c2.Instance = I2C2;
   hi2c2.Init.Timing = 0x00602173;
   hi2c2.Init.OwnAddress1 = 0;
   hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
   hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
   hi2c2.Init.OwnAddress2 = 0;
   hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
   hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
   hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
   if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
      Error_Handler();
   }
   /** Configure Analogue filter
    */
   if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
         != HAL_OK) {
      Error_Handler();
   }
   /** Configure Digital filter
    */
   if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
      Error_Handler();
   }
   /* USER CODE BEGIN I2C2_Init 2 */

   /* USER CODE END I2C2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

   /* USER CODE BEGIN TIM3_Init 0 */

   /* USER CODE END TIM3_Init 0 */

   TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
   TIM_MasterConfigTypeDef sMasterConfig = { 0 };
   TIM_OC_InitTypeDef sConfigOC = { 0 };

   /* USER CODE BEGIN TIM3_Init 1 */

   /* USER CODE END TIM3_Init 1 */
   htim3.Instance = TIM3;
   htim3.Init.Prescaler = 63;
   htim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;
   htim3.Init.Period = 468;
   htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
   if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
      Error_Handler();
   }
   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
   if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
      Error_Handler();
   }
   if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
      Error_Handler();
   }
   sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
   if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
         != HAL_OK) {
      Error_Handler();
   }
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = 40;
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
      Error_Handler();
   }
   /* USER CODE BEGIN TIM3_Init 2 */

   /* USER CODE END TIM3_Init 2 */
   HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

   /* USER CODE BEGIN USART3_Init 0 */

   /* USER CODE END USART3_Init 0 */

   /* USER CODE BEGIN USART3_Init 1 */

   /* USER CODE END USART3_Init 1 */
   huart3.Instance = USART3;
   huart3.Init.BaudRate = 115200;
   huart3.Init.WordLength = UART_WORDLENGTH_8B;
   huart3.Init.StopBits = UART_STOPBITS_1;
   huart3.Init.Parity = UART_PARITY_NONE;
   huart3.Init.Mode = UART_MODE_TX_RX;
   huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart3.Init.OverSampling = UART_OVERSAMPLING_16;
   huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
   huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
   huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
   if (HAL_UART_Init(&huart3) != HAL_OK) {
      Error_Handler();
   }
   if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8)
         != HAL_OK) {
      Error_Handler();
   }
   if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8)
         != HAL_OK) {
      Error_Handler();
   }
   if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
      Error_Handler();
   }
   /* USER CODE BEGIN USART3_Init 2 */

   /* USER CODE END USART3_Init 2 */

}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void) {

   /* USER CODE BEGIN USB_OTG_FS_Init 0 */

   /* USER CODE END USB_OTG_FS_Init 0 */

   /* USER CODE BEGIN USB_OTG_FS_Init 1 */

   /* USER CODE END USB_OTG_FS_Init 1 */
   hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
   hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
   hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
   hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
   hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
   hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
   hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
   hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
   hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
   hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
   hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
   if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
      Error_Handler();
   }
   /* USER CODE BEGIN USB_OTG_FS_Init 2 */

   /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
   GPIO_InitTypeDef GPIO_InitStruct = { 0 };

   /* GPIO Ports Clock Enable */
   __HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_GPIOF_CLK_ENABLE();
   __HAL_RCC_GPIOH_CLK_ENABLE();
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();
   __HAL_RCC_GPIOD_CLK_ENABLE();
   __HAL_RCC_GPIOG_CLK_ENABLE();
   __HAL_RCC_GPIOE_CLK_ENABLE();

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin, GPIO_PIN_RESET);

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin,
         GPIO_PIN_RESET);

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(DIR_Pin_GPIO_Port, DIR_Pin_Pin, GPIO_PIN_RESET);

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

   /*Configure GPIO pin : B1_Pin */
   GPIO_InitStruct.Pin = B1_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

   /*Configure GPIO pins : LD1_Pin LD3_Pin */
   GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
   GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

   /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
   GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

   /*Configure GPIO pin : DIR_Pin_Pin */
   GPIO_InitStruct.Pin = DIR_Pin_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(DIR_Pin_GPIO_Port, &GPIO_InitStruct);

   /*Configure GPIO pin : LD2_Pin */
   GPIO_InitStruct.Pin = LD2_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Stop_Timer(TIM_TypeDef *TIMx){
   TIMx->CR1 &= ~(TIM_CR1_CEN) ;
}
void Start_Timer(TIM_TypeDef *TIMx){
   TIMx->CR1 |= TIM_CR1_CEN;
}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
   // Interrupt service routine for TIMER3
   // Counts pulses until the desired number have been sent
   // Then it disables the timer to end the output.

   // !! Only works now because TIM3 is the only PWM timer.
   // !! The complete solution is to check TIM status bits to see
   // !! who issued the interrupt.

   Stop_Timer(TIM3); // Freeze counter at zero

   Stepper->UpdateStep();
   if (Stepper->MoveComplete()) {
      Stepper->Stop();
   } else {
      // TODO acceleration stuff here
      Start_Timer(TIM3); // Keep counting
   }

}

void Move_Stepper(enum Direction dir, int full_turns, int next_number) {
   /** Function to move the stepper. Tell it the direction of rotation,
    * the number of complete revolutions to add, and the final number to land on.
    *
    * This function calculates the number of steps required to reach the final
    * position, sends the pulses, and waits for the stepper to finish moving.
    *
    * Then it compares the position measured from the Hall Effect sensor to
    * the position given by the move command. If they match, then the dial's position
    * is updated. If they don't match, then we have an error or an open state.
    */

   Stepper->Move(Dial->CalculateSteps(dir, full_turns, next_number), dir);
   while (Stepper->Status() == Running) {} // Wait for move to finish
   HAL_Delay(DELAY_MS); // Brief delay after move to allow mechanical settling.

   // [TODO] Compare expected and measured positions here.

   Dial->UpdatePosition(next_number); // Expected and measured are in agreement.

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
