/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32wlxx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* PB0: TCXO VDD (Must be Analog mode before HSE BYPASS PWR is enabled) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PA4, PA5: RF Switch Control (CTRL1, CTRL2) */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */

/**
 * @brief  SubGHz MSP Initialization.
 *         Enables the SUBGHZ peripheral clock and configures the
 *         SUBGHZ_Radio_IRQn interrupt used for Tx/Rx done signals.
 * @param  hsubghz  SUBGHZ handle pointer (unused, single instance)
 */
void HAL_SUBGHZ_MspInit(SUBGHZ_HandleTypeDef *hsubghz)
{
    (void)hsubghz;

    /* Enable SUBGHZ peripheral clock */
    __HAL_RCC_SUBGHZSPI_CLK_ENABLE();

    /* SUBGHZ_Radio_IRQn — priority 0 (highest), sub-priority 0 */
    HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);
}

/**
 * @brief  SubGHz MSP De-Initialization.
 *         Disables clock and NVIC IRQ for the SUBGHZ peripheral.
 * @param  hsubghz  SUBGHZ handle pointer (unused)
 */
void HAL_SUBGHZ_MspDeInit(SUBGHZ_HandleTypeDef *hsubghz)
{
    (void)hsubghz;

    __HAL_RCC_SUBGHZSPI_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(SUBGHZ_Radio_IRQn);
}

/* USER CODE END 1 */
