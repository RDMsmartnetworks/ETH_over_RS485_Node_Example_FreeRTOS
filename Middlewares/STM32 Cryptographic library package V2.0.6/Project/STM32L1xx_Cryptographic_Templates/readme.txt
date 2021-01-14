/**
  @page AES  AES128_CTR Example
  
  @verbatim
  ******************** (C) COPYRIGHT 2012 STMicroelectronics *******************
  * @file    AES128_CTR/readme.txt 
  * @author  MCD Application Team
  * @version V2.0.4
  * @date    02-Augusr-2012
  * @brief   Description of the "AES-128 in CTR mode" example.
  ******************************************************************************
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
  * limitations under the Licens
  * 
  ******************************************************************************
   @endverbatim

@par Example Description
This example describes how to use the STM32 Crypto Library AES-128 algorithm to

  - Encrypt DATA with AES-128 in CTR mode,
  - Decrypt DATA with AES-128 in CTR mode.
NOTE: NIST Special Publication 800-38A 2001 Edition..................  
The example shows the different steps :
  - Initialization,
  - Data encryption/decryption
  - Finalization 
to perform AES encryption/decryption in CTR mode.

AES algorithm routines can return AES_SUCCESS in case of successful 
operations or one of the following error codes in case of fail:
 
  - AES_ERR_BAD_INPUT_SIZE,
  - AES_ERR_BAD_OPERATION,
  - AES_ERR_BAD_CONTEXT,
  - AES_ERR_BAD_PARAMETER

User application needs to check on returned value to determine which action will
be run in both fail and success cases.

@par Directory contents

  - AES/AES128_CTR/main.c          Main program


@note The System Clock configuration are handled in the Cryptographic library
      Project template for each devices, and can be easily customized 
      to your own configuration.

@par Hardware and Software environment

  - This example runs on:
    - STM32F0xx Devices.
  
  - This example has been tested with STMicroelectronics STM30518-EVAL 
    evaluation boards and doesn't depend on external hardware on the evaluation
    boards.
    
@par How to use it? 

In order to make the program work, you must do the following :
 - Copy main.c source file from this example folder to the template folder under
   Project\STM32F0xx_Cryptographic_Template.
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example

    
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
