/**
  ******************************************************************************
  * @file    RAND/Random_NumberGenerator/main.c
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
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Entropy String. Uniformly distributed random bit string 1*/
uint8_t entropy_data[32] = {
                             0x9d, 0x20, 0x1a, 0x18, 0x9b, 0x6d, 0x1a, 0xa7, 0x0e,
                             0x79, 0x57, 0x6f, 0x36, 0xb6, 0xaa, 0x88, 0x55, 0xfd,
                             0x4a, 0x7f, 0x97, 0xe9, 0x71, 0x69, 0xb6, 0x60, 0x88,
                             0x78, 0xe1, 0x9c, 0x8b, 0xa5
                           };
/* Nonce. Non repeating sequence, such as a timestamp */
uint8_t nonce[] = {0xFE, 0xA9, 0x96, 0xD4, 0x62, 0xC5};
/* Personalization String */
uint8_t personalization_String[] = {0x1E, 0x6C, 0x7B, 0x82, 0xE5, 0xA5, 0x71, 0x8D};
/* Structure that will keep the Random State */
RNGstate_stt RNGstate;

/* DRBG type */
#ifndef USE_HW_RNG
  int32_t DRBGtype = C_DRBG_AES128;
#else
  int32_t DRBGtype = C_HW_RNG;
#endif

/* Array that will be filled with random bytes */
uint8_t RandomString[32] = {0, };


/* Private function prototypes -----------------------------------------------*/
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

  /* Strucutre used to call the Random engine inizialition */
  RNGinitInput_stt RNGinit_st;
  int32_t status = RNG_SUCCESS;

  /* DeInitialize STM32 Cryptographic Library */
  Crypto_DeInit();

  /* Set the values of EntropyData, Nonce, Personalization String and their sizes inside the RNGinit_st structure */
  RNGinit_st.pmEntropyData = entropy_data;
  RNGinit_st.mEntropyDataSize = sizeof(entropy_data);
  RNGinit_st.pmNonce =  nonce;
  RNGinit_st.mNonceSize = sizeof( nonce );
  RNGinit_st.pmPersData = personalization_String;
  RNGinit_st.mPersDataSize = sizeof( personalization_String );

  status = RNGinit(&RNGinit_st, DRBGtype,  &RNGstate);
  if  ( status == RNG_SUCCESS )
  {
    /* The Random engine has been initialized, the status is in RNGstate */

    /* Now fill the random string with random bytes */
    status = RNGgenBytes(&RNGstate, NULL, RandomString, sizeof(RandomString));

    if (status == RNG_SUCCESS)
    {
      /* Random Generated Succefully, free the state before returning */
      RNGfree(&RNGstate);
    }
    else
    {
      /* Add application traintment in case of randomization not success possible values of status:
       * RNG_ERR_BAD_PARAMETER, RNG_ERR_UNINIT_STATE
       */
    }

  }
  else
  {
    /* Add application traintment in case of randomization not success possible values of status:
     * RNG_ERR_BAD_ENTROPY_SIZE, RNG_ERR_BAD_PERS_STRING_SIZE
     */
  }

  /* Infinite loop */
  while (1)
  {}
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
