// ****************************************************************************
/// \file      main.c
///
/// \brief     MAIN Module
///
/// \details   mainloop
///
/// \author    Nico Korn
///
/// \version   1.0.0.0
///
/// \date      14012021
/// 
/// \copyright Copyright 2021 Reichle & De-Massari AG
///            
///            Permission is hereby granted, free of charge, to any person 
///            obtaining a copy of this software and associated documentation 
///            files (the "Software"), to deal in the Software without 
///            restriction, including without limitation the rights to use, 
///            copy, modify, merge, publish, distribute, sublicense, and/or sell
///            copies of the Software, and to permit persons to whom the 
///            Software is furnished to do so, subject to the following 
///            conditions:
///            
///            The above copyright notice and this permission notice shall be 
///            included in all copies or substantial portions of the Software.
///            
///            THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
///            EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
///            OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
///            NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
///            HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
///            WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
///            FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR 
///            OTHER DEALINGS IN THE SOFTWARE.
///
/// \pre       
///
/// \bug       
///
/// \warning    
///
/// \todo      
///
// ****************************************************************************

// Include ********************************************************************
#include <stdio.h>

#include "FreeRTOS.h" 
#include "task.h"     
#include "queue.h"    
#include "timers.h"   
#include "semphr.h"  

#include "main.h"
#include "bus_uart_tcp.h"

// defines ********************************************************************
#define TIMEOUT 100

// Global variables ***********************************************************

// Private variables **********************************************************
static IWDG_HandleTypeDef   IwdgHandle;

// Private function prototypes ************************************************
static void SystemClock_Config(void);
static void main_initWatchdog( void );

//------------------------------------------------------------------------------
/// \brief     Application entry point
///
/// \param     none
///
/// \return    none
int main( void )
{
   // Reset of all peripherals, Initializes the Flash interface and the Systick.
   HAL_Init();
   
   // Configure the system clock
   SystemClock_Config();
   
   // HAL_Init() sets the default priority grouping
   // to 4U (3-premption, 1-sub priority). FreeRTOS 
   // highly recommend to set the priority grouping 
   // to only 0 i.e. preempt priorities only.
   // webLink: https://www.freertos.org/RTOS-Cortex-M3-M4.html
   NVIC_SetPriorityGrouping(0U);
   
   // start the tcp ip stack
   init_tcp_ip_stack();

   // Start FreeRTOS Kernel Scheduler
   vTaskStartScheduler();
  
   // Infinite loop
   while(1);
}

//------------------------------------------------------------------------------
/// \brief     Clock configuration function 
///
/// \param     none
///
/// \return    none
static void SystemClock_Config( void )
{
   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  
   /** Configure the main internal regulator output voltage 
   */
   __HAL_RCC_PWR_CLK_ENABLE();
   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
   
   /** Initializes the CPU, AHB and APB busses clocks 
   */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
   RCC_OscInitStruct.PLL.PLLM = 12;
   RCC_OscInitStruct.PLL.PLLN = 100;
   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
   RCC_OscInitStruct.PLL.PLLQ = 2;
   RCC_OscInitStruct.PLL.PLLR = 2;
   HAL_RCC_OscConfig(&RCC_OscInitStruct);
       
   /** Initializes the CPU, AHB and APB busses clocks 
   */
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
   HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
}

//------------------------------------------------------------------------------
/// \brief     initialisation of the watchdog
///            Set counter reload value to obtain 1 sec. IWDG TimeOut.
///            IWDG counter clock Frequency = uwLsiFreq
///            Set Prescaler to 32 (IWDG_PRESCALER_32)
///            Timeout Period = (Reload Counter Value * 32) / uwLsiFreq
///            So Set Reload Counter Value = (1 * uwLsiFreq) / 32 */
///
/// \param     none
///
/// \return    none
static void main_initWatchdog( void )
{
  IwdgHandle.Instance = IWDG;
  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_128;
  IwdgHandle.Init.Reload = ( (12*LSI_VALUE) / 32 ); //~10 seconds
  HAL_IWDG_Init(&IwdgHandle);
}

//------------------------------------------------------------------------------
/// \brief     refresh of the watchdog
///
/// \param     none
///
/// \return    none
void main_refreshWatchdog( void )
{
   HAL_IWDG_Refresh(&IwdgHandle);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
