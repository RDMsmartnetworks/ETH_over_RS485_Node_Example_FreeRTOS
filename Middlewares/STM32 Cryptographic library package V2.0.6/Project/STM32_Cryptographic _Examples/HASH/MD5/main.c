/**
  ******************************************************************************
  * @file    HASH/MD5/main.c
  * @author  MCD Application Team
  * @version V2.0.6
  * @date    25-June-2013
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

#include <stdio.h>
#include "crypto.h"

/** @addtogroup STM32_Crypto_Examples
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
/* Private define ------------------------------------------------------------*/
#define HASH_LENGTH 16
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const uint8_t InputMessage[] =  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
/* string length only, without '\0' end of string marker */
uint32_t InputLength = (sizeof(InputMessage) - 1);
uint8_t MessageDigest[CRL_MD5_SIZE];
int32_t MessageDigestLength = 0;

const uint8_t Expected_OutputMessage[HASH_LENGTH] =
  {
    0xd1, 0x74, 0xab, 0x98,
    0xd2, 0x77, 0xd9, 0xf5,
    0xa5, 0x61, 0x1c, 0x2c,
    0x9f, 0x41, 0x9d, 0x9f
  };


/* Private function prototypes -----------------------------------------------*/
int32_t STM32_MD5_HASH_DigestCompute(uint8_t* InputMessage,
                               uint32_t InputMessageLength,
                               uint8_t *MessageDigest,
                               int32_t* MessageDigestLength);
TestStatus Buffercmp(const uint8_t* pBuffer,
                     uint8_t* pBuffer1,
                     uint16_t BufferLength);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c, system_stm32l1xx.c, system_stm32f0xx.c, 
       system_stm32f2xx.c, system_stm32f30x.c, system_stm32f37x.c, or
       system_stm32f4xx.c file depending on device.
     */
     
  int32_t status = HASH_SUCCESS;

  /* DeInitialize STM32 Cryptographic Library */
  Crypto_DeInit();

  status = STM32_MD5_HASH_DigestCompute((uint8_t*)InputMessage,
                                  InputLength,
                                  (uint8_t*)MessageDigest,
                                  &MessageDigestLength);
  if (status == HASH_SUCCESS)
  {
    /* add application traintment in case of hash success */
    if (Buffercmp(Expected_OutputMessage, MessageDigest, HASH_LENGTH) == PASSED)
    {
      /* add application traintment in case of MD5 operation is passed */
      
    }
    else
    {
      /* add application traintment in case of MD5 operation is failed */
    }
  }
  else
  {
    /* Add application traintment in case of hash not success possible values of status:
     * HASH_ERR_BAD_PARAMETER, HASH_ERR_BAD_CONTEXT, HASH_ERR_BAD_OPERATION
     */
  }

  /* Infinite loop */
  while (1)
  {}
}

/**
  * @brief  MD5 HASH digest compute example.
  * @param  InputMessage: pointer to input message to be hashed.
  * @param  InputMessageLength: input data message length in byte.
  * @param  MessageDigest: pointer to output parameter that will handle message digest
  * @param  MessageDigestLength: pointer to output digest length.
  * @retval error status: can be HASH_SUCCESS if success or one of
  *         HASH_ERR_BAD_PARAMETER, HASH_ERR_BAD_CONTEXT,
  *         HASH_ERR_BAD_OPERATION if error occured.
  */
int32_t STM32_MD5_HASH_DigestCompute(uint8_t* InputMessage, uint32_t InputMessageLength,
                               uint8_t *MessageDigest, int32_t* MessageDigestLength)
{
  MD5ctx_stt P_pMD5ctx;
  uint32_t error_status = HASH_SUCCESS;

  /* Set the size of the desired hash digest */
  P_pMD5ctx.mTagSize = CRL_MD5_SIZE;

  /* Set flag field to default value */
  P_pMD5ctx.mFlags = E_HASH_DEFAULT;

  error_status = MD5_Init(&P_pMD5ctx);

  /* check for initialization errors */
  if (error_status == HASH_SUCCESS)
  {
    /* Add data to be hashed */
    error_status = MD5_Append(&P_pMD5ctx,
                              InputMessage,
                              InputMessageLength);

    if (error_status == HASH_SUCCESS)
    {
      /* retrieve */
      error_status = MD5_Finish(&P_pMD5ctx, MessageDigest, MessageDigestLength);
    }
  }

  return error_status;
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer, pBuffer1: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer identical to pBuffer1
  *         FAILED: pBuffer differs from pBuffer1
  */
TestStatus Buffercmp(const uint8_t* pBuffer, uint8_t* pBuffer1, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer != *pBuffer1)
    {
      return FAILED;
    }

    pBuffer++;
    pBuffer1++;
  }

  return PASSED;
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
