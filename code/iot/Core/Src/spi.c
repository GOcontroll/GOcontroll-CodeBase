/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */

  /* GPDMA1 clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
    PeriphClkInitStruct.PLL2.PLL2Source = RCC_PLL2_SOURCE_HSE;
    PeriphClkInitStruct.PLL2.PLL2M = 6;
    PeriphClkInitStruct.PLL2.PLL2N = 250;
    PeriphClkInitStruct.PLL2.PLL2P = 4;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2_VCIRANGE_1;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2_VCORANGE_WIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.PLL2.PLL2ClockOut = RCC_PLL2_DIVP;
    PeriphClkInitStruct.Spi1ClockSelection = RCC_SPI1CLKSOURCE_PLL2P;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
	HAL_GPIO_WritePin(GPIOA, SPI_MOD_SCK_Pin|SPI_MOD_MISO_Pin|SPI_MOD_MOSI_Pin, GPIO_PIN_RESET);
	  
    GPIO_InitStruct.Pin = SPI_MOD_SCK_Pin|SPI_MOD_MISO_Pin|SPI_MOD_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* SPI1 RX: GPDMA1 Channel 0 */
  hdma_spi1_rx.Instance                   = GPDMA1_Channel0;
  hdma_spi1_rx.Init.Request               = GPDMA1_REQUEST_SPI1_RX;
  hdma_spi1_rx.Init.BlkHWRequest          = DMA_BREQ_SINGLE_BURST;
  hdma_spi1_rx.Init.Direction             = DMA_PERIPH_TO_MEMORY;
  hdma_spi1_rx.Init.SrcInc                = DMA_SINC_FIXED;
  hdma_spi1_rx.Init.DestInc               = DMA_DINC_INCREMENTED;
  hdma_spi1_rx.Init.SrcDataWidth          = DMA_SRC_DATAWIDTH_BYTE;
  hdma_spi1_rx.Init.DestDataWidth         = DMA_DEST_DATAWIDTH_BYTE;
  hdma_spi1_rx.Init.Priority              = DMA_LOW_PRIORITY_HIGH_WEIGHT;
  hdma_spi1_rx.Init.SrcBurstLength        = 1;
  hdma_spi1_rx.Init.DestBurstLength       = 1;
  hdma_spi1_rx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  hdma_spi1_rx.Init.TransferEventMode     = DMA_TCEM_BLOCK_TRANSFER;
  hdma_spi1_rx.Init.Mode                  = DMA_NORMAL;
  if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK) { Error_Handler(); }
  __HAL_LINKDMA(spiHandle, hdmarx, hdma_spi1_rx);
  HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

  /* SPI1 TX: GPDMA1 Channel 1 */
  hdma_spi1_tx.Instance                   = GPDMA1_Channel1;
  hdma_spi1_tx.Init.Request               = GPDMA1_REQUEST_SPI1_TX;
  hdma_spi1_tx.Init.BlkHWRequest          = DMA_BREQ_SINGLE_BURST;
  hdma_spi1_tx.Init.Direction             = DMA_MEMORY_TO_PERIPH;
  hdma_spi1_tx.Init.SrcInc                = DMA_SINC_INCREMENTED;
  hdma_spi1_tx.Init.DestInc               = DMA_DINC_FIXED;
  hdma_spi1_tx.Init.SrcDataWidth          = DMA_SRC_DATAWIDTH_BYTE;
  hdma_spi1_tx.Init.DestDataWidth         = DMA_DEST_DATAWIDTH_BYTE;
  hdma_spi1_tx.Init.Priority              = DMA_LOW_PRIORITY_HIGH_WEIGHT;
  hdma_spi1_tx.Init.SrcBurstLength        = 1;
  hdma_spi1_tx.Init.DestBurstLength       = 1;
  hdma_spi1_tx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  hdma_spi1_tx.Init.TransferEventMode     = DMA_TCEM_BLOCK_TRANSFER;
  hdma_spi1_tx.Init.Mode                  = DMA_NORMAL;
  if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK) { Error_Handler(); }
  __HAL_LINKDMA(spiHandle, hdmatx, hdma_spi1_tx);
  HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

  /* SPI1 peripheral IRQ (handles EOT after DMA, plus UDR/FRE/MODF errors) */
  HAL_NVIC_SetPriority(SPI1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);
    HAL_NVIC_DisableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_DisableIRQ(GPDMA1_Channel1_IRQn);
    HAL_NVIC_DisableIRQ(SPI1_IRQn);

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, SPI_MOD_SCK_Pin|SPI_MOD_MISO_Pin|SPI_MOD_MOSI_Pin);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
