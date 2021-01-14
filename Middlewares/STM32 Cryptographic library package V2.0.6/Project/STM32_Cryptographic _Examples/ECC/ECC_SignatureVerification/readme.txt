/**
  @page ECC  ECC_SignatureVerification Example
  
  @verbatim
  ******************** (C) COPYRIGHT 2013 STMicroelectronics *******************
  * @file    ECC_SignatureVerification/readme.txt 
  * @author  MCD Application Team
  * @version V2.0.6
  * @date    25-June-2013
  * @brief   Description of the "ECC_SignatureVerification" example.
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
  * limitations under the License
  * 
  ******************************************************************************
   @endverbatim

@par Example Description

This example describes how to use the STM32 Cryptographic Library ECC algorithm 
in example of an ECC signature verification with Parameters for Elliptic Curve 
P-192, P-256 and P-384.

ECC starts by proceeding with ECC signature generation and initilizing the 
Elliptic Curve. When Public Key is validated, signature object will be 
initialized and prepare the structure for the ECDSA signature verification based
on SHA-256.

ECC functions can return ECC_SUCCESS in case of successful ECC
operation or one of the following error codes in case of fail: 
  - ECC_ERR_BAD_PARAMETER,
  - ECC_ERR_BAD_CONTEXT,
  - HASH_ERR_BAD_OPERATION.
  
User application needs to check on returned value to determine which action will
be run in both fail and success cases.

@par Directory contents

  - ECC_SignatureVerification/main.c          Main program
  
@note The System Clock configuration are handled in the Cryptographic library
      Project, and can be easily customized to your own configuration.

@par Hardware and Software environment

 - This example runs on:

    - STM32F10x Devices. 
    - STM32L1xx Devices.
    - STM32F2xx Devices.    
    - STM32F40/41x Devices.    
    - STM32F30x/31x Devices.    
    - STM32F37x/38x Devices.
      
 - This example has been tested with STMicroelectronics STM3210B-EVAL (Medium-
    Density), STM3210E-EVAL (High-Density and XL-Density), STM3210C-EVAL 
    (STM32F10x Connectivity-Line), STM32 Value line discovery kit (STM32F10x 
    Medium-Density Value line), STM32100E-EVAL (High-Density Value line), 
    STM3220G-EVAL, STM32L152-EVAL (STM32L1xx Ultra Low Power Medium-Density), 
    STM32F4 discovery kit, STM32 20-21-45-46 G-EVAL, STM32F373C-EVAL, STM32F3 
    Discovery kit and STM32F0 discovery Kit and they don't depend on external 
    hardware on the evaluation boards.
  
      
 - This example has been tested with STMicroelectronics STM3210B-EVAL (Medium-
    Density), STM3210E-EVAL (High-Density and XL-Density), STM3210C-EVAL 
    (STM32F10x Connectivity-Line), STM32 Value line discovery kit (STM32F10x 
    Medium-Density Value line), STM32100E-EVAL (High-Density Value line), 
    STM3220G-EVAL, STM32L152-EVAL (STM32L1xx Ultra Low Power Medium-Density), 
    STM32F4 discovery kit, STM32 20-21-45-46 G-EVAL, STM32F373C-EVAL, STM32F3 
    Discovery kit and STM32F0 discovery Kit and they don't depend on external 
    hardware on the evaluation boards.
    
@par How to use it? 

In order to make the program work, you must do the following :
 - Copy main.c source file from this example folder to the template folder under
   Project\STM32F10x_Cryptographic_Templates when using STM32F10x series.
   Project\STM32F2xx_Cryptographic_Templates when using STM32F2xx series.
   Project\STM32L1xx_Cryptographic_Templates when using STM32L1xx series.
   Project\STM32F0xx_Cryptographic_Templates when using STM32F0xx series.
   Project\STM32F4xx_Cryptographic_Templates when using STM32F4xx series.
   Project\STM32F37x_Cryptographic_Templates when using STM32F37x/38xseries.
   Project\STM32F30x_Cryptographic_Templates when using STM32F30x/31x series.
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example
    
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */