/**
  ******************************************************************************
  * @file    stm32f4xx_hal_crc.c
  * @author  MCD Application Team
  * @brief   CRC HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Cyclic Redundancy Check (CRC) peripheral:
  *           + Initialization and de-initialization functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
  *
  @verbatim
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    [..]
         (+) Enable CRC AHB clock using __HAL_RCC_CRC_CLK_ENABLE();
         (+) Initialize CRC calculator
             (++) specify generating polynomial (peripheral default or non-default one)
             (++) specify initialization value (peripheral default or non-default one)
             (++) specify input data format
             (++) specify input or output data inversion mode if any
         (+) Use HAL_CRC_Accumulate() function to compute the CRC value of the
             input data buffer starting with the previously computed CRC as
             initialization value
         (+) Use HAL_CRC_Calculate() function to compute the CRC value of the
             input data buffer starting with the defined initialization value
             (default or non-default) to initiate CRC calculation

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup CRC CRC
  * @brief CRC HAL module driver.
  * @{
  */

#ifdef HAL_CRC_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static uint32_t CRC_Handle_8(CRC_HandleTypeDef *hcrc, uint8_t pBuffer[], uint32_t BufferLength);
static uint32_t revbit(uint32_t uData);
/* Exported functions --------------------------------------------------------*/

/** @defgroup CRC_Exported_Functions CRC Exported Functions
  * @{
  */

/** @defgroup CRC_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions.
 *
@verbatim
 ===============================================================================
            ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize the CRC according to the specified parameters
          in the CRC_InitTypeDef and create the associated handle
      (+) DeInitialize the CRC peripheral
      (+) Initialize the CRC MSP (MCU Specific Package)
      (+) DeInitialize the CRC MSP

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the CRC according to the specified
  *         parameters in the CRC_InitTypeDef and create the associated handle.
  * @param  hcrc CRC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRC_Init(CRC_HandleTypeDef *hcrc)
{
  /* Check the CRC handle allocation */
  if (hcrc == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_CRC_ALL_INSTANCE(hcrc->Instance));

  if (hcrc->State == HAL_CRC_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hcrc->Lock = HAL_UNLOCKED;
    /* Init the low level hardware */
    HAL_CRC_MspInit(hcrc);
  }

  hcrc->State = HAL_CRC_STATE_BUSY;

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_READY;

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  DeInitialize the CRC peripheral.
  * @param  hcrc CRC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRC_DeInit(CRC_HandleTypeDef *hcrc)
{
  /* Check the CRC handle allocation */
  if (hcrc == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_CRC_ALL_INSTANCE(hcrc->Instance));

  /* Check the CRC peripheral state */
  if (hcrc->State == HAL_CRC_STATE_BUSY)
  {
    return HAL_BUSY;
  }

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_BUSY;

  /* Reset CRC calculation unit */
  __HAL_CRC_DR_RESET(hcrc);

  /* Reset IDR register content */
  CLEAR_BIT(hcrc->Instance->IDR, CRC_IDR_IDR);

  /* DeInit the low level hardware */
  HAL_CRC_MspDeInit(hcrc);

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_RESET;

  /* Process unlocked */
  __HAL_UNLOCK(hcrc);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRC MSP.
  * @param  hcrc CRC handle
  * @retval None
  */
__weak void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcrc);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_CRC_MspInit can be implemented in the user file
   */
}

/**
  * @brief  DeInitialize the CRC MSP.
  * @param  hcrc CRC handle
  * @retval None
  */
__weak void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcrc);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_CRC_MspDeInit can be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup CRC_Exported_Functions_Group2 Peripheral Control functions
 *  @brief    management functions.
 *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) compute the 32-bit CRC value of a 32-bit data buffer
          using combination of the previous CRC value and the new one.

       [..]  or

      (+) compute the 32-bit CRC value of a 32-bit data buffer
          independently of the previous CRC value.

@endverbatim
  * @{
  */

/**
  * @brief  Compute the 32-bit CRC value of a 32-bit data buffer
  *         starting with the previously computed CRC as initialization value.
  * @param  hcrc CRC handle
  * @param  pBuffer pointer to the input data buffer.
  * @param  BufferLength input data buffer length (number of uint32_t words).
  * @retval uint32_t CRC (returned value LSBs for CRC shorter than 32 bits)
  */
uint32_t HAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength)
{
  uint32_t index;      /* CRC input data buffer index */
  uint32_t temp = 0U;  /* CRC output (read from hcrc->Instance->DR register) */

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_BUSY;

  /* Enter Data to the CRC calculator */
  for (index = 0U; index < BufferLength; index++)
  {
    hcrc->Instance->DR = pBuffer[index];
  }
  temp = hcrc->Instance->DR;

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_READY;

  /* Return the CRC computed value */
  return temp;
}

/**
  * @brief  Compute the 32-bit CRC value of a 32-bit data buffer
  *         starting with hcrc->Instance->INIT as initialization value.
  * @param  hcrc CRC handle
  * @param  pBuffer pointer to the input data buffer.
  * @param  BufferLength input data buffer length (number of uint32_t words).
  * @retval uint32_t CRC (returned value LSBs for CRC shorter than 32 bits)
  */
uint32_t HAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength)
{
  static uint32_t index;      /* CRC input data buffer index */
  volatile uint32_t temp = 0U;  /* CRC output (read from hcrc->Instance->DR register) */
  volatile  uint8_t *pBuffer8B;

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_BUSY;

  /* Reset CRC Calculation Unit (hcrc->Instance->INIT is
  *  written in hcrc->Instance->DR) */
  __HAL_CRC_DR_RESET(hcrc);
  
  /* Enter 32-bit input data to the CRC calculator */
  /*
  for (index = 0U; index < BufferLength; index++)
  {
    *(__IO uint8_t *)(__IO void *)(&hcrc->Instance->DR) = *(__IO uint8_t *)(__IO void *)(&pBuffer[index]);
  }
  temp = hcrc->Instance->DR;
*/
  temp = CRC_Handle_8(hcrc, (uint8_t *)pBuffer, BufferLength);

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_READY;

  /* Return the CRC computed value */
  //temp ^= 0xFFFFFFFF;
  return temp;
}

/**
  * @brief  Enter 8-bit input data to the CRC calculator.
  *         Specific data handling to optimize processing time.
  * @param  hcrc CRC handle
  * @param  pBuffer pointer to the input data buffer
  * @param  BufferLength input data buffer length
  * @retval uint32_t CRC (returned value LSBs for CRC shorter than 32 bits)
  */
static uint32_t CRC_Handle_8(CRC_HandleTypeDef *hcrc, uint8_t pBuffer[], uint32_t BufferLength)
{
  volatile uint16_t  data;
  volatile uint16_t  *pReg;
  volatile uint32_t  CRC32;
  uint32_t   byte1;
  uint32_t   byte2;
  uint32_t   byte3;
  uint32_t   byte4;
  uint32_t   crcInputWord;

  /* Processing time optimization: 4 bytes are entered in a row with a single word write,
   * last bytes must be carefully fed to the CRC calculator to ensure a correct type
   * handling by the peripheral */

    /* Processing time optimization: 4 bytes are entered in a row with a single word write,
   * last bytes must be carefully fed to the CRC calculator to ensure a correct type
   * handling by the peripheral */
  
  /* handle n * 4 byte chunks + 0 byte */
   if ((BufferLength % 4U) == 0U)
   {
      for (uint32_t i = 0U; i < (BufferLength / 4U); i++)
      {
         hcrc->Instance->DR = ((uint32_t)pBuffer[4U * i] << 24U) | \
                              ((uint32_t)pBuffer[(4U * i) + 1U] << 16U) | \
                              ((uint32_t)pBuffer[(4U * i) + 2U] << 8U)  | \
                              (uint32_t)pBuffer[(4U * i) + 3U];
      }
      CRC32 = hcrc->Instance->DR;
   }
   
   /* handle n * 4 byte chunks + 1 byte */
   if ((BufferLength % 4U) == 1U)
   {
      // fill the first 3 Bytes with FF's
      hcrc->Instance->DR = ((uint32_t)0xFF << 24U) | \
                           ((uint32_t)0xFF << 16U) | \
                           ((uint32_t)0xFF << 8U)  | \
                           (uint32_t)pBuffer[0];
      
      if(BufferLength==1)
      {
         // just invert the first 3 values and finish the crc computation
         uint8_t *temp;
         CRC32 = hcrc->Instance->DR;
         temp = (uint8_t*)&CRC32;
         temp[1] = ~temp[1];
         temp[2] = ~temp[2];
         temp[3] = ~temp[3];
      }
      else
      {
         // second byte
         byte1 = ((uint32_t)pBuffer[1U] << 24U);
         byte2 = ((uint32_t)pBuffer[2U] << 16U);
         byte3 = ((uint32_t)pBuffer[3U] << 8U);
         byte4 = (uint32_t)pBuffer[4U];
         
         byte1 = ~byte1;
         byte2 = ~byte2;
         byte3 = ~byte3;
         
         byte1 = byte1 & 0xFF000000;
         byte2 = byte2 & 0x00FF0000;
         byte3 = byte3 & 0x0000FF00;
            
         crcInputWord = byte1 | byte2 | byte3 | byte4;
            
         hcrc->Instance->DR = crcInputWord;
         
         // from th third byte on just feed the crc wordwise
         for (uint32_t i = 1U; i < (BufferLength / 4U); i++)
         {
            hcrc->Instance->DR = ((uint32_t)pBuffer[(4U * i) + 1U] << 24U) | \
                                 ((uint32_t)pBuffer[(4U * i) + 2U] << 16U) | \
                                 ((uint32_t)pBuffer[(4U * i) + 3U] << 8U)  | \
                                 (uint32_t)pBuffer[(4U * i) + 4U];
         }
         
         // read the final crc value
         CRC32 = hcrc->Instance->DR;
      }
   }
   
   /* handle n * 4 byte chunks + 2 byte */
   if ((BufferLength % 4U) == 2U)
   {
      // fill the first 2 Bytes with FF's
      hcrc->Instance->DR = ((uint32_t)0xFF << 24U) | \
                           ((uint32_t)0xFF << 16U) | \
                           ((uint32_t)pBuffer[0] << 8U)  | \
                           (uint32_t)pBuffer[1];
      
      if(BufferLength==2)
      {
         // just invert the first 2 values and finish the crc computation
         uint8_t *temp;
         CRC32 = hcrc->Instance->DR;
         temp = (uint8_t*)&CRC32;
         temp[2] = ~temp[2];
         temp[3] = ~temp[3];
      }
      else
      {
         // second byte
         byte1 = ((uint32_t)pBuffer[2U] << 24U);
         byte2 = ((uint32_t)pBuffer[3U] << 16U);
         byte3 = ((uint32_t)pBuffer[4U] << 8U);
         byte4 = (uint32_t)pBuffer[5U];
         
         byte1 = ~byte1;
         byte2 = ~byte2;
         
         byte1 = byte1 & 0xFF000000;
         byte2 = byte2 & 0x00FF0000;
            
         crcInputWord = byte1 | byte2 | byte3 | byte4;
            
         hcrc->Instance->DR = crcInputWord;
         
         // from th third byte on just feed the crc wordwise
         for (uint32_t i = 1U; i < (BufferLength / 4U); i++)
         {
            hcrc->Instance->DR = ((uint32_t)pBuffer[(4U * i) + 2U] << 24U) | \
                                 ((uint32_t)pBuffer[(4U * i) + 3U] << 16U) | \
                                 ((uint32_t)pBuffer[(4U * i) + 4U] << 8U)  | \
                                  (uint32_t)pBuffer[(4U * i) + 5U];
         }
         
         // read the final crc value
         CRC32 = hcrc->Instance->DR;
      }
   }
   
   /* handle n * 4 byte chunks + 3 byte */
   if ((BufferLength % 4U) == 3U)
   {
      // fill the first 1 Byte with FF
      hcrc->Instance->DR = ((uint32_t)0xFF << 24U) | \
                           ((uint32_t)pBuffer[0] << 16U) | \
                           ((uint32_t)pBuffer[1] << 8U)  | \
                           (uint32_t)pBuffer[2];
      
      if(BufferLength==3)
      {
         // just invert the first 2 values and finish the crc computation
         uint8_t *temp;
         CRC32 = hcrc->Instance->DR;
         temp = (uint8_t*)&CRC32;
         temp[3] = ~temp[3];
      }
      else
      {
         // second byte
         byte1 = ((uint32_t)pBuffer[3U] << 24U);
         byte2 = ((uint32_t)pBuffer[4U] << 16U);
         byte3 = ((uint32_t)pBuffer[5U] << 8U);
         byte4 = (uint32_t)pBuffer[6U];
         
         byte1 = ~byte1;
         
         byte1 = byte1 & 0xFF000000;
            
         crcInputWord = byte1 | byte2 | byte3 | byte4;
            
         hcrc->Instance->DR = crcInputWord;
         
         // from th third byte on just feed the crc wordwise
         for (uint32_t i = 1U; i < (BufferLength / 4U); i++)
         {
            hcrc->Instance->DR = ((uint32_t)pBuffer[(4U * i) + 3U] << 24U) | \
                                 ((uint32_t)pBuffer[(4U * i) + 4U] << 16U) | \
                                 ((uint32_t)pBuffer[(4U * i) + 5U] << 8U)  | \
                                  (uint32_t)pBuffer[(4U * i) + 6U];
         }
         
         // read the final crc value
         CRC32 = hcrc->Instance->DR;
      }
   }

  /* Return the CRC computed value */
  return CRC32;
}

/**
  * @}
  */

/** @defgroup CRC_Exported_Functions_Group3 Peripheral State functions
 *  @brief    Peripheral State functions.
 *
@verbatim
 ===============================================================================
                      ##### Peripheral State functions #####
 ===============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Return the CRC handle state.
  * @param  hcrc CRC handle
  * @retval HAL state
  */
HAL_CRC_StateTypeDef HAL_CRC_GetState(CRC_HandleTypeDef *hcrc)
{
  /* Return CRC handle state */
  return hcrc->State;
}

uint32_t TM_CRC_Calculate8(uint8_t* arr, uint32_t count) {
	/* Reset CRC data register if necessary */
   CRC->CR = CRC_CR_RESET;
	
	/* Calculate CRC */
	while (count--)
   {
		/* Set new value */
		CRC->DR = *arr++;
	}
	
	/* Return data */
	return CRC->DR;
}
#endif /* HAL_CRC_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
