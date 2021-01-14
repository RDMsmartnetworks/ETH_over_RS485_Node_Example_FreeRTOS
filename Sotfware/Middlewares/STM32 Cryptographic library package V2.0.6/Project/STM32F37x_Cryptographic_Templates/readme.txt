/**
  @page AES  AES128_CTR Example
  
  @verbatim
  ******************** (C) COPYRIGHT 2013 STMicroelectronics *******************
  * @file    AES128_CTR/readme.txt 
  * @author  MCD Application Team
  * @version V2.0.6
  * @date    25-June-2013
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
This example describes how to use the STM32 Cryptographic Library AES algorithm
and the different steps (Initialization/Append data/Finalization) to perform AES
128 bits Encryption/Decryption processes for both CTR modes .

This example is split into two parts:
The first part is to encrypt DATA in CTR mode and compare between the encrypted 
data and expected encrypted data is done to check that the input data have been
correctly encrypted.

The Second part is to decrypt DATA in CTRs mode and compare between the decrypted 
data and expected decrypted data is done to check that the input data have been
correctly decrypted.

@note NIST vectors examples for AES CTR are taken from:
"Recommendation for Block Cipher Modes of Operation, 2001 Edition"
Available at:
 http://csrc.nist.gov/publications/nistpubs/800-38a/sp800-38a.pdf

@par Directory contents

  - AES/AES128_CTR/main.c          Main program


@note The System Clock configuration are handled in the Cryptographic library
      Project template for each devices, and can be easily customized 
      to your own configuration.

@par Hardware and Software environment

- This example runs on:
    - STM32F10x Connectivity line, High-Density, Medium-Density, 
      XL-Density, High-Density Value line, Medium-Density Value line, 
      Low-Density and Low-Density Value line Devices.

    - STM32L1xx Ultra Low Power Medium-Density Devices Devices.

    - STM32F2xx Devices.
    
    - STM32F40/1x Devices.
    
    - STM32F30x/31x Devices.
    
    - STM32F37x/38x Devices.

@note  - When using optimized STM32 Cryptographic library based on hardware 
        Cryptographic peripheral DMA2 Stream 5 and DMA2 Stream 6 are reserved to 
        perform data tranfer to/from Cryptographic block. 
         
       - Cryptographic and DMA2 peripheral clocks are enabled with algorithm 
         initialization.
         
       - User application should disable these clocks when no more 
         encryption/decryption will be performed.    
  
 - This example has been tested with STMicroelectronics STM3210B-EVAL (Medium-Density), 
    STM3210E-EVAL (High-Density and XL-Density), STM3210C-EVAL (STM32F10x Connectivity-Line), 
    STM32 Value line discovery kit (STM32F10x Medium-Density Value line), 
    STM32100E-EVAL (High-Density Value line), STM3220G-EVAL, 
    STM32L152-EVAL (STM32L1xx Ultra Low Power Medium-Density), STM32F4 discovery kit
    and STM32 20-21-45-46 G-EVAL evaluation boards and doesn't depend on external 
    hardware on the evaluation boards.
    
 - This example has been tested with STMicroelectronics STM3210B-EVAL (Medium-Density), 
    STM3210E-EVAL (High-Density and XL-Density), STM3210C-EVAL (STM32F10x Connectivity-Line), 
    STM32 Value line discovery kit (STM32F10x Medium-Density Value line), 
    STM32100E-EVAL (High-Density Value line), STM3220G-EVAL, 
    STM32L152-EVAL (STM32L1xx Ultra Low Power Medium-Density), STM32F4 discovery
    kit, STM32 20-21-45-46 G-EVAL, STM32F373C-EVAL, STM32F3 Discovery kit and 
    STM32F0 discovery Kit and doesn't depend on external hardware on the evaluation 
    boards.
    
@par How to use it? 

In order to make the program work, you must do the following :
 - Copy main.c source file from this example folder to the template folder under
   Project\STM32F10x_Cryptographic_Templates when using STM32F10x series.
   Project\STM32F2xx_Cryptographic_Templates when using STM32F2xx series.
   Project\STM32L1xx_Cryptographic_Templates when using STM32L1xx series.
   Project\STM32F0xx_Cryptographic_Templates when using STM32L4xx series.
   Project\STM32F4xx_Cryptographic_Templates when using STM32L4xx series.
   Project\STM32F37x_Cryptographic_Templates when using STM32F37x/38xseries.
   Project\STM32F30x_Cryptographic_Templates when using STM32F30x/31x series.
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example
    
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */