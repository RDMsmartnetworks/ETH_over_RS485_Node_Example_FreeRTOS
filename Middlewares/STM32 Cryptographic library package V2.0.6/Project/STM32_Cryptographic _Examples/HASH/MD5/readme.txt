/**
  @page HASH_MD5_Example MD5 Hash Example
  
  @verbatim
  ******************** (C) COPYRIGHT 2013 STMicroelectronics *******************
  * @file    HASH/MD5/readme.txt 
  * @author  MCD Application Team
  * @version V2.0.6
  * @date    25-June-2013
  * @brief   Description of the MD5 Hash example.
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
This example describes how to use the STM32 Cryptographic Library MD5 Hash  
algorithm to generate message digest.

The MD5 Hash algorithm is configured to generate 16 bytes hash digest 
(CRL_MD5_SIZE) and in default mode (E_HASH_DEFAULT).

MD5 hash routines can return HASH_SUCCESS in case of successful hash operation
or one of the following error codes in case of fail: 
  - HASH_ERR_BAD_PARAMETER,
  - HASH_ERR_BAD_CONTEXT, 
  - HASH_ERR_BAD_OPERATION.

A comparison between the generated digest data and expected digest data is done 
to check that the input data have been correctly hashed.

@note vectors examples for HASH/MD5 are taken from RFC Publication:
"The MD5 Message-Digest Algorithm April 1992"
Available at:
  http://www.rfc-editor.org/rfc/pdfrfc/rfc1321.txt.pdf

@par Directory contents

  - HASH/MD5/main.c          Main program
  
@note The System Clock configuration are handled in the Cryptographic library
      Project template for each devices, and can be easily customized 
      to your own configuration.

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