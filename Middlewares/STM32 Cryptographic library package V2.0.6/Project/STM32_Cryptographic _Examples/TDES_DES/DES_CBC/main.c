/**
  ******************************************************************************
  * @file    TDES_DES/DES_CBC/main.c
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "crypto.h"

/** @addtogroup STM32_Crypto_Examples
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
/* Private define ------------------------------------------------------------*/
#define PLAINTEXT_LENGTH 24
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const uint8_t Plaintext[PLAINTEXT_LENGTH] =
  {
    0x4e, 0x6f, 0x77, 0x20, 0x69, 0x73, 0x20, 0x74, 0x68, 0x65, 0x20, 0x74, 0x69,
    0x6d, 0x65, 0x20, 0x66, 0x6f, 0x72, 0x20, 0x61, 0x6c, 0x6c, 0x20  
  };

/* Key to be used for DES encryption/decryption */
uint8_t Key[CRL_DES_KEY] =
  {
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef
  };

/* Initialization Vector */
uint8_t IV[CRL_DES_BLOCK] = 
  {
    0x12, 0x34, 0x56, 0x78, 0x90, 0xab, 0xcd, 0xef
  };

/* Buffer to store the output data */
uint8_t OutputMessage[PLAINTEXT_LENGTH];

/* Size of the output data */
uint32_t OutputMessageLength = 0;

const uint8_t Expected_Ciphertext[PLAINTEXT_LENGTH] =
  {
    0xe5, 0xc7, 0xcd, 0xde, 0x87, 0x2b, 0xf2, 0x7c, 0x43, 0xe9, 0x34, 0x00, 0x8c,
    0x38, 0x9c, 0x0f, 0x68, 0x37, 0x88, 0x49, 0x9a, 0x7c, 0x05, 0xf6  
  };

/* Private function prototypes -----------------------------------------------*/
int32_t STM32_DES_CBC_Encrypt(uint8_t*  InputMessage,
                        uint32_t  InputMessageLength,
                        uint8_t  *DES_Key,
                        uint8_t  *InitializationVector,
                        uint32_t  IvLength,
                        uint8_t  *OutputMessage,
                        uint32_t *OutputMessageLength);

int32_t STM32_DES_CBC_Decrypt(uint8_t*  InputMessage,
                        uint32_t  InputMessageLength,
                        uint8_t  *DES_Key,
                        uint8_t  *InitializationVector,
                        uint32_t  IvLength,
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
     
  int32_t status = DES_SUCCESS;

  /* DeInitialize STM32 Cryptographic Library */
   Crypto_DeInit();

  /* Encrypt DATA with DES in CBC mode */
  status = STM32_DES_CBC_Encrypt( (uint8_t *) Plaintext, PLAINTEXT_LENGTH, Key, IV, sizeof(IV), OutputMessage,
                            &OutputMessageLength);
  if (status == DES_SUCCESS)
  {
    if (Buffercmp(Expected_Ciphertext, OutputMessage, PLAINTEXT_LENGTH) == PASSED)
    {
      /* add application traintment in case of DES CBC encrption is passed */   
     
    }
    else
    {
      /* add application traintment in case of DES CBC encrption is failed */
    }
  }
  else
  {
    /* Add application traintment in case of encryption/decryption not success possible values
       *  of status:
       * DES_ERR_BAD_INPUT_SIZE, DES_ERR_BAD_OPERATION, DES_ERR_BAD_CONTEXT, DES_ERR_BAD_PARAMETER
       */
  }
  /* Decrypt DATA with DES in CBC mode */
  status = STM32_DES_CBC_Decrypt( (uint8_t *) Expected_Ciphertext, PLAINTEXT_LENGTH, Key, IV, sizeof(IV), OutputMessage,
                            &OutputMessageLength);
  if (status == DES_SUCCESS)
  {
    if (Buffercmp(Plaintext, OutputMessage, PLAINTEXT_LENGTH) == PASSED)
    {
      /* add application traintment in case of DES CBC encrption is passed */    
		
    }
    else
    {
      /* add application traintment in case of DES CBC encrption is failed */
    }
  }
  else
  {
    /* Add application traintment in case of encryption/decryption not success possible values
       *  of status:
       * DES_ERR_BAD_INPUT_SIZE, DES_ERR_BAD_OPERATION, DES_ERR_BAD_CONTEXT, DES_ERR_BAD_PARAMETER
       */
  }
  /* Infinite loop */
  while (1)
  {}
}

/**
  * @brief  DES CBC Encryption example.
  * @param  InputMessage: pointer to input message to be encrypted.
  * @param  InputMessageLength: input data message length in byte.
  * @param  DES_Key: pointer to the DES key to be used in the operation
  * @param  InitializationVector: pointer to the Initialization Vector (IV)
  * @param  IvLength: IV length in bytes.
  * @param  OutputMessage: pointer to output parameter that will handle the encrypted message
  * @param  OutputMessageLength: pointer to encrypted message length.
  * @retval error status: can be DES_SUCCESS if success or one of
  *         DES_ERR_BAD_INPUT_SIZE, DES_ERR_BAD_OPERATION, DES_ERR_BAD_CONTEXT
  *         DES_ERR_BAD_PARAMETER if error occured.
  */
int32_t STM32_DES_CBC_Encrypt(uint8_t* InputMessage,
                        uint32_t InputMessageLength,
                        uint8_t  *DES_Key,
                        uint8_t  *InitializationVector,
                        uint32_t  IvLength,
                        uint8_t  *OutputMessage,
                        uint32_t *OutputMessageLength)
{
  DESCBCctx_stt DESctx;

  uint32_t error_status = DES_SUCCESS;

  int32_t outputLength = 0;

  /* Set flag field to default value */
  DESctx.mFlags = E_SK_DEFAULT;

  /* Set iv size field to IvLength*/
  DESctx.mIvSize = IvLength;

  /* Initialize the operation, by passing the key.
   * Third parameter is NULL because CBC doesn't use any IV */
  error_status = DES_CBC_Encrypt_Init(&DESctx, DES_Key, InitializationVector );

  /* check for initialization errors */
  if (error_status == DES_SUCCESS)
  {
    /* Encrypt Data */
    error_status = DES_CBC_Encrypt_Append(&DESctx,
                                          InputMessage,
                                          InputMessageLength,
                                          OutputMessage,
                                          &outputLength);

    if (error_status == DES_SUCCESS)
    {
      /* Write the number of data written*/
      *OutputMessageLength = outputLength;
      /* Do the Finalization */
      error_status = DES_CBC_Encrypt_Finish(&DESctx, OutputMessage + *OutputMessageLength, &outputLength);
      /* Add data written to the information to be returned */
      *OutputMessageLength += outputLength;
    }
  }

  return error_status;
}


/**
  * @brief  DES CBC Decryption example.
  * @param  InputMessage: pointer to input message to be decrypted.
  * @param  InputMessageLength: input data message length in byte.
  * @param  DES192_Key: pointer to the DES key to be used in the operation
  * @param  InitializationVector: pointer to the Initialization Vector (IV)
  * @param  IvLength: IV length in bytes.
  * @param  OutputMessage: pointer to output parameter that will handle the decrypted message
  * @param  OutputMessageLength: pointer to decrypted message length.
  * @retval error status: can be DES_SUCCESS if success or one of
  *         DES_ERR_BAD_INPUT_SIZE, DES_ERR_BAD_OPERATION, DES_ERR_BAD_CONTEXT
  *         DES_ERR_BAD_PARAMETER if error occured.
  */
int32_t STM32_DES_CBC_Decrypt(uint8_t* InputMessage,
                        uint32_t InputMessageLength,
                        uint8_t  *DES_Key,
                        uint8_t  *InitializationVector,
                        uint32_t  IvLength,
                        uint8_t  *OutputMessage,
                        uint32_t *OutputMessageLength)
{
  DESCBCctx_stt DESctx;

  uint32_t error_status = DES_SUCCESS;

  int32_t outputLength = 0;

  /* Set flag field to default value */
  DESctx.mFlags = E_SK_DEFAULT;

  /* Set iv size field to IvLength*/
  DESctx.mIvSize = IvLength;

  /* Initialize the operation, by passing the key.
   * Third parameter is NULL because CBC doesn't use any IV */
  error_status = DES_CBC_Decrypt_Init(&DESctx, DES_Key, InitializationVector );

  /* check for initialization errors */
  if (error_status == DES_SUCCESS)
  {
    /* Decrypt Data */
    error_status = DES_CBC_Decrypt_Append(&DESctx,
                                          InputMessage,
                                          InputMessageLength,
                                          OutputMessage,
                                          &outputLength);

    if (error_status == DES_SUCCESS)
    {
      /* Write the number of data written*/
      *OutputMessageLength = outputLength;
      /* Do the Finalization */
      error_status = DES_CBC_Decrypt_Finish(&DESctx, OutputMessage + *OutputMessageLength, &outputLength);
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


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
