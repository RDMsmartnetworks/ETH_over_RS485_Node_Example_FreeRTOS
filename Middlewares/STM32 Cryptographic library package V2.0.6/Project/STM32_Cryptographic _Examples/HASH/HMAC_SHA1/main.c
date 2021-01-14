/**
  ******************************************************************************
  * @file    HASH/HMAC_SHA1/main.c
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
#define HMAC_LENGTH 20
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const uint8_t InputMessage[] =  "Sample #1";
/* string length only, without '\0' end of string marker */
uint32_t InputLength = (sizeof(InputMessage) - 1);

uint8_t HMAC_Key[] =
  {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
    0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13,
    0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
    0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
    0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31,
    0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b,
    0x3c, 0x3d, 0x3e, 0x3f
  };

/* string length only, without '\0' end of string marker */
uint32_t KeyLength = (sizeof(HMAC_Key));

uint8_t MAC[CRL_SHA1_SIZE];
int32_t MACLength = 0;

const uint8_t Expected_OutputMessage[HMAC_LENGTH] =
  {
    0x4f, 0x4c, 0xa3, 0xd5, 0xd6,
    0x8b, 0xa7, 0xcc, 0x0a, 0x12,
    0x08, 0xc9, 0xc6, 0x1e, 0x9c,
    0x5d, 0xa0, 0x40, 0x3c, 0x0a
  };

/* Private function prototypes -----------------------------------------------*/
int32_t STM32_SHA1_HMAC_Compute(uint8_t* InputMessage,
                          uint32_t InputMessageLength,
                          uint8_t *HMAC_key,
                          uint32_t HMAC_keyLength,
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

  status = STM32_SHA1_HMAC_Compute((uint8_t*)InputMessage,
                             InputLength,
                             HMAC_Key,
                             KeyLength,
                             (uint8_t*)MAC,
                             &MACLength);
  if (status == HASH_SUCCESS)
  {
    /* add application traintment in case of hash success */
    if (Buffercmp(Expected_OutputMessage, MAC, HMAC_LENGTH) == PASSED)
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
  * @brief  SHA1 HMAC compute example.
  * @param  InputMessage: pointer to input message to be hashed.
  * @param  InputMessageLength: input data message length in byte.
  * @param  HMAC_key: pointer to key used in the HMAC computation
  * @param  HMAC_keyLength: HMAC key length in byte.
  * @param  MessageDigest: pointer to output parameter that will handle message digest
  * @param  MessageDigestLength: pointer to output digest length.
  * @retval error status: can be HASH_SUCCESS if success or one of
  *         HASH_ERR_BAD_PARAMETER, HASH_ERR_BAD_CONTEXT,
  *         HASH_ERR_BAD_OPERATION if error occured.
  */
int32_t STM32_SHA1_HMAC_Compute(uint8_t* InputMessage,
                          uint32_t InputMessageLength,
                          uint8_t *HMAC_key,
                          uint32_t HMAC_keyLength,
                          uint8_t *MessageDigest,
                          int32_t* MessageDigestLength)
{
  HMAC_SHA1ctx_stt HMAC_SHA1ctx;
  uint32_t error_status = HASH_SUCCESS;

  /* Set the size of the desired MAC*/
  HMAC_SHA1ctx.mTagSize = CRL_SHA1_SIZE;

  /* Set flag field to default value */
  HMAC_SHA1ctx.mFlags = E_HASH_DEFAULT;

  /* Set the key pointer in the context*/
  HMAC_SHA1ctx.pmKey = HMAC_key;

  /* Set the size of the key */
  HMAC_SHA1ctx.mKeySize = HMAC_keyLength;

  /* Initialize the context */
  error_status = HMAC_SHA1_Init(&HMAC_SHA1ctx);

  /* check for initialization errors */
  if (error_status == HASH_SUCCESS)
  {
    /* Add data to be hashed */
    error_status = HMAC_SHA1_Append(&HMAC_SHA1ctx,
                                    InputMessage,
                                    InputMessageLength);

    if (error_status == HASH_SUCCESS)
    {
      /* retrieve */
      error_status = HMAC_SHA1_Finish(&HMAC_SHA1ctx, MessageDigest, MessageDigestLength);
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
