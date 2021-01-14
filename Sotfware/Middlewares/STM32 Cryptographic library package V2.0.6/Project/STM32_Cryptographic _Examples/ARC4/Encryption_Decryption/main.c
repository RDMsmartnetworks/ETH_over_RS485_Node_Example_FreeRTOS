/**
  ******************************************************************************
  * @file    ARC4/Encryption_Decryption/main.c
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
#define ARC4_LENGTH 32
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const uint8_t InputMessage[32] =
  {
    0x00,
  };

uint32_t InputLength = sizeof(InputMessage);

/* Key to be used for ARC4 encryption/decryption */
uint8_t Key[5] =
  {
    0x01, 0x02, 0x03, 0x04, 0x05
  };

/* Buffer to store the output data */
uint8_t OutputMessage[ARC4_LENGTH];

/* Size of the output data */
uint32_t OutputMessageLength = 0;

const uint8_t Expected_OutputMessage[ARC4_LENGTH] =
  {
    0xb2, 0x39, 0x63, 0x05,
    0xf0, 0x3d, 0xc0, 0x27,
    0xcc, 0xc3, 0x52, 0x4a,
    0x0a, 0x11, 0x18, 0xa8,
    0x69, 0x82, 0x94, 0x4f,
    0x18, 0xfc, 0x82, 0xd5,
    0x89, 0xc4, 0x03, 0xa4,
    0x7a, 0x0d, 0x09, 0x19
  };

/* Private function prototypes -----------------------------------------------*/
int32_t STM32_ARC4_Encrypt(uint8_t* InputMessage,
                     uint32_t InputMessageLength,
                     uint8_t  *ARC4_Key,
                     uint32_t KeyLength,
                     uint8_t  *OutputMessage,
                     uint32_t *OutputMessageLength);

int32_t STM32_ARC4_Decrypt(uint8_t* InputMessage,
                     uint32_t InputMessageLength,
                     uint8_t  *ARC4_Key,
                     uint32_t KeyLength,
                     uint8_t  *OutputMessage,
                     uint32_t *OutputMessageLength);

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
  int32_t status = AES_SUCCESS;

  /* DeInitialize STM32 Cryptographic Library */
   Crypto_DeInit();

  /* Encrypt DATA with ARC4 */
  status = STM32_ARC4_Encrypt( (uint8_t *) InputMessage, InputLength, Key, (uint32_t) sizeof(Key), OutputMessage,
                         &OutputMessageLength);
  if (status == ARC4_SUCCESS)
  {

    if (Buffercmp(Expected_OutputMessage, OutputMessage, OutputMessageLength) == PASSED)
    {
      /* add application traintment in case of ARC4 success */

    }
    else
    {
      /* add application traintment in case of ARC4 failed */

    }
  }
  else
  {
    /* Add application traintment in case of encryption/decryption not success possible values
      *  of status:
      * ARC4_ERR_BAD_OPERATION, ARC4_ERR_BAD_CONTEXT, ARC4_ERR_BAD_PARAMETER
      */  
  }

  /* Data has been encrypted, now decrypt it.
   * Decrypt DATA with ARC4 */
  status = STM32_ARC4_Decrypt( (uint8_t *) Expected_OutputMessage, ARC4_LENGTH, Key, (uint32_t) sizeof(Key),
                         OutputMessage, &OutputMessageLength);
  if (status == ARC4_SUCCESS)
  {

    if (Buffercmp(InputMessage, OutputMessage, InputLength) == PASSED)
    {
      /* add application traintment in case of ARC4 success */

    }
    else
    {
      /* add application traintment in case of ARC4 failed */
    
    }
  }
  else
  {
    /* Add application traintment in case of encryption/decryption not success possible values
      *  of status:
      * ARC4_ERR_BAD_OPERATION, ARC4_ERR_BAD_CONTEXT, ARC4_ERR_BAD_PARAMETER
      */
         
  }

  /* Infinite loop */
  while (1)
  {}
}



/**
  * @brief  ARC4 Encryption example.
  * @param  InputMessage: pointer to input message to be encrypted.
  * @param  InputMessageLength: input data message length in byte.
  * @param  ARC4_Key: pointer to the ARC4 key to be used in the operation
  * @param  KeyLength: ARC4 key length in byte
  * @param  OutputMessage: pointer to output parameter that will handle the encrypted message
  * @param  OutputMessageLength: pointer to encrypted message length.
  * @retval error status: can be ARC4_SUCCESS if success or one of
  *         ARC4_ERR_BAD_OPERATION, ARC4_ERR_BAD_CONTEXT, ARC4_ERR_BAD_PARAMETER
  *         if error occured.
  */
int32_t STM32_ARC4_Encrypt(uint8_t* InputMessage,
                     uint32_t InputMessageLength,
                     uint8_t  *ARC4_Key,
                     uint32_t KeyLength,
                     uint8_t  *OutputMessage,
                     uint32_t *OutputMessageLength)
{
  ARC4ctx_stt ARC4ctx;

  uint32_t error_status = ARC4_SUCCESS;

  int32_t outputLength = 0;

  /* Set flag field to default value */
  ARC4ctx.mFlags = E_SK_DEFAULT;

  /* Set key length in the context */
  ARC4ctx.mKeySize = KeyLength;

  /* Initialize the operation, by passing the key.
   * Third parameter is NULL because ARC4 doesn't use any IV */
  error_status = ARC4_Encrypt_Init(&ARC4ctx, ARC4_Key, NULL );

  /* check for initialization errors */
  if (error_status == ARC4_SUCCESS)
  {
    /* Encrypt Data */
    error_status = ARC4_Encrypt_Append(&ARC4ctx,
                                       InputMessage,
                                       InputMessageLength,
                                       OutputMessage,
                                       &outputLength);

    if (error_status == ARC4_SUCCESS)
    {
      /* Write the number of data written*/
      *OutputMessageLength = outputLength;
      /* Do the Finalization */
      error_status = ARC4_Encrypt_Finish(&ARC4ctx, OutputMessage + *OutputMessageLength, &outputLength);
      /* Add data written to the information to be returned */
      *OutputMessageLength += outputLength;
    }
  }

  return error_status;
}


/**
  * @brief  ARC4 Decryption example.
  * @param  InputMessage: pointer to input message to be decrypted.
  * @param  InputMessageLength: input data message length in byte.
  * @param  ARC4_Key: pointer to the ARC4 key to be used in the operation
  * @param  KeyLength: ARC4 key length in byte
  * @param  OutputMessage: pointer to output parameter that will handle the decrypted message
  * @param  OutputMessageLength: pointer to decrypted message length.
  * @retval error status: can be ARC4_SUCCESS if success or one of
  *         ARC4_ERR_BAD_OPERATION, ARC4_ERR_BAD_CONTEXT, ARC4_ERR_BAD_PARAMETER
  *         if error occured.
  */
int32_t STM32_ARC4_Decrypt(uint8_t* InputMessage,
                     uint32_t InputMessageLength,
                     uint8_t  *ARC4_Key,
                     uint32_t KeyLength,
                     uint8_t  *OutputMessage,
                     uint32_t *OutputMessageLength)
{
  ARC4ctx_stt ARC4ctx;

  uint32_t error_status = ARC4_SUCCESS;

  int32_t outputLength = 0;

  /* Set flag field to default value */
  ARC4ctx.mFlags = E_SK_DEFAULT;

  /* Set key length in the context */
  ARC4ctx.mKeySize = KeyLength;

  /* Initialize the operation, by passing the key.
   * Third parameter is NULL because ECB doesn't use any IV */
  error_status = ARC4_Decrypt_Init(&ARC4ctx, ARC4_Key, NULL );

  /* check for initialization errors */
  if (error_status == ARC4_SUCCESS)
  {
    /* Decrypt Data */
    error_status = ARC4_Decrypt_Append(&ARC4ctx,
                                       InputMessage,
                                       InputMessageLength,
                                       OutputMessage,
                                       &outputLength);

    if (error_status == ARC4_SUCCESS)
    {
      /* Write the number of data written*/
      *OutputMessageLength = outputLength;
      /* Do the Finalization */
      error_status = ARC4_Decrypt_Finish(&ARC4ctx, OutputMessage + *OutputMessageLength, &outputLength);
      /* Add data written to the information to be returned */
      *OutputMessageLength += outputLength;
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
