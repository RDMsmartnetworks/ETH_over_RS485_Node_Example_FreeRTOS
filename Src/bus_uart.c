// ****************************************************************************
/// \file      bus_uart.c
///
/// \brief     bus uart module
///
/// \details   Module used to initialise bus uart peripherals. Module also 
///            contents the functions for receiving and transmitting data. 
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
#include  <stdlib.h>
#include "FreeRTOS.h"
#include "networkinterface.h"
#include "bus_uart.h"
#include "bus_uart_tcp.h"

// Private define *************************************************************
#define IOBUFFERSIZE             ( ETHSIZE )
#define FRAMEGAPTIME             ( 1000u ) // 1=0.1 us

// Private types **************************************************************

// Private variables **********************************************************
static BUS_UART_RX_t             busuartRx;                        
static uint8_t                   txBuffer1[IOBUFFERSIZE] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAB};     
static volatile FlagStatus       framegapTimeoutFlag     = SET;          
static volatile FlagStatus       randomTimeoutFlag       = SET;      
static CRC_HandleTypeDef         CRC_Handle;     
static FlagStatus                busEthFrameDetected     = RESET;
static const uint8_t             macBroadcast[]          = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; 
static const uint8_t             macLLMNR[]              = { 0x01, 0x00, 0x5E, 0x00, 0x00, 0xFC }; 
static const uint8_t             ipLLMNR[]               = { 224, 0, 0, 252 }; 
static const uint8_t             arpCode[]               = { 0x08, 0x06 };
static const uint8_t             preAmbleSFD[]           = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAB};
static uint8_t*                  macFramePointer;
static uint16_t                  macFrameLength;
BUS_UART_INPUT_FIFO_HANDLE_t     inputFrameHandle;
static const uint8_t*            stackMacAddress;
// for debugging
static uint32_t                  setRandomWaitCounter;
static uint32_t                  debugcounter;
static BUS_UART_STATISTIC_t      bus_uart_statistic;


// Private function prototypes ************************************************
static uint8_t                   bus_uart_macCheck             ( uint8_t* pointerToMacAddress);
static uint8_t                   bus_uart_ipCheck              ( uint8_t* pointerToIpAddress );
static void                      bus_uart_startRandomTimeoutTx ( void );
static void                      bus_uart_testCRC              ( void );
static uint8_t                   bus_initUartDmaTimer          ( void );

// Global variables     *******************************************************
UART_HandleTypeDef               UART_Bus_Handle;
DMA_HandleTypeDef 					HDMA_BUS_rx;
DMA_HandleTypeDef 					HDMA_BUS_tx;
extern const uint8_t             ucMACAddressFLASH[];
extern const uint8_t             ucIPAddressFLASH[];
TIM_HandleTypeDef                TIM7_Handle;
TIM_HandleTypeDef                TIM3_Handle;

// ----------------------------------------------------------------------------
/// \brief     initialise bus uart peripherals
///
/// \return    0 (init fail), 1 (init success)
uint8_t bus_uart_init( void )
{
   // initialise the random number generator
   bus_uart_tcp_rngInit();
   
   // CRC Init
   __HAL_RCC_CRC_CLK_ENABLE();
   CRC_Handle.Instance = CRC;
   HAL_CRC_Init(&CRC_Handle);
   bus_uart_testCRC();
   
   // get mac address pointer from the stack
   stackMacAddress = FreeRTOS_GetMACAddress();
   
   // reset statistics
   bus_uart_statistic.counterFrame1Byte                  = 0;  
   bus_uart_statistic.counterFrameTooLong                = 0;   
   bus_uart_statistic.counterFrameTooShort               = 0;          
   bus_uart_statistic.counterMACAlienFrame               = 0;          
   bus_uart_statistic.counterMACBroadcast                = 0;
   bus_uart_statistic.counterMACLLMNR                    = 0;
   bus_uart_statistic.counterARPAlienFrame               = 0;          
   bus_uart_statistic.counterFrameInvalidPreamble        = 0;
   bus_uart_statistic.counterFrameInvalidCRC             = 0;
   bus_uart_statistic.counterValidFrame                  = 0;
   
   // init the frame input fifo
   // set all fifo entities to invalid, since no data has been received yet
   for( uint8_t i=0; i<INPUTFRAMEFIFOSIZE; i++ )
   {
      inputFrameHandle.inputFrameFifo[i].frameSize       = 0;
   }
   inputFrameHandle.inputFrameCurrentFifoSize            = 0;
   inputFrameHandle.inputFrameFifoFullFlag               = RESET;
   inputFrameHandle.inputFrameHeadIndex                  = 0;
   inputFrameHandle.inputFrameTailIndex                  = 0;

   
   // init RX/TX pin
   __HAL_RCC_GPIOD_CLK_ENABLE();
   GPIO_InitTypeDef              GPIO_Init_Struct;
   GPIO_Init_Struct.Pin          = UART_PIN_BUS_RX|UART_PIN_BUS_TX;
   GPIO_Init_Struct.Mode         = GPIO_MODE_AF_PP;
   GPIO_Init_Struct.Pull         = GPIO_PULLUP;
   GPIO_Init_Struct.Speed        = GPIO_SPEED_FREQ_VERY_HIGH;
   GPIO_Init_Struct.Alternate    = GPIO_AF7_USART2;
   HAL_GPIO_Init(UART_GPIO, &GPIO_Init_Struct);
   
   // init RTS pin
   GPIO_Init_Struct.Pin          = UART_PIN_BUS_RTS;
   GPIO_Init_Struct.Mode         = GPIO_MODE_OUTPUT_PP;
   HAL_GPIO_Init(UART_GPIO, &GPIO_Init_Struct);
   
   // init CTS pin
   GPIO_Init_Struct.Pin          = UART_PIN_BUS_CTS;
   GPIO_Init_Struct.Mode         = GPIO_MODE_OUTPUT_PP;
   HAL_GPIO_Init(UART_GPIO, &GPIO_Init_Struct);
   
   // init BE pin
   GPIO_Init_Struct.Pin          = UART_PIN_BUS_BE;
   GPIO_Init_Struct.Mode         = GPIO_MODE_OUTPUT_PP;
   HAL_GPIO_Init(UART_GPIO, &GPIO_Init_Struct);
   
   // enable inverting driver
   HAL_GPIO_WritePin(UART_GPIO, UART_PIN_BUS_BE, GPIO_PIN_RESET);
   
   // initialize uart, dma and timer
   bus_initUartDmaTimer();

   // start to receive
   busuartRx.bufferSize = IOBUFFERSIZE;      // set input output buffer size on the rx type struct
   busuartRx.buffer = inputFrameHandle.inputFrameFifo[0].frameArray;
   bus_uart_receive( &UART_Bus_Handle, busuartRx.buffer, busuartRx.bufferSize );
   
   return 1;
}

// ----------------------------------------------------------------------------
/// \brief     Deinitialise bus uart peripherals
///
/// \return    0 (init fail), 1 (init success)
uint8_t bus_uart_deinit( void )
{
   bus_uart_tcp_rngDeInit();
   __HAL_RCC_GPIOD_CLK_DISABLE();
   HAL_GPIO_DeInit(UART_GPIO, UART_PIN_BUS_RX|UART_PIN_BUS_TX|UART_PIN_BUS_RTS|UART_PIN_BUS_BE|UART_PIN_BUS_CTS);
   __HAL_RCC_USART2_CLK_DISABLE();
   HAL_UART_DeInit(&UART_Bus_Handle);
	__HAL_RCC_DMA1_CLK_DISABLE();
   HAL_DMA_DeInit(&HDMA_BUS_rx);
   HAL_DMA_DeInit(&HDMA_BUS_tx);
    __HAL_RCC_CRC_CLK_DISABLE();
   HAL_CRC_DeInit(&CRC_Handle);
   __HAL_RCC_TIM7_CLK_DISABLE();
   HAL_TIM_Base_DeInit(&TIM7_Handle);
   __HAL_RCC_TIM3_CLK_DISABLE();
   HAL_TIM_Base_DeInit(&TIM3_Handle);
   return 1;
}

//------------------------------------------------------------------------------
/// \brief     Setup DMA and Timers for Bus uart          
///
/// \param     none
///
/// \return    0 = error, 1 = success
static uint8_t bus_initUartDmaTimer( void )
{
      // ini UART
   __HAL_RCC_USART2_CLK_ENABLE();
   UART_Bus_Handle.Instance            = USART2;
   UART_Bus_Handle.Init.BaudRate       = UART_BUS_BAUDRATE;
   UART_Bus_Handle.Init.WordLength     = UART_WORDLENGTH_8B;
   UART_Bus_Handle.Init.StopBits       = UART_STOPBITS_1;
   UART_Bus_Handle.Init.Parity         = UART_PARITY_NONE;
   UART_Bus_Handle.Init.Mode           = UART_MODE_TX_RX;
   UART_Bus_Handle.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
   UART_Bus_Handle.Init.OverSampling   = UART_OVERSAMPLING_8;
   if (HAL_UART_Init(&UART_Bus_Handle) != HAL_OK)
   {
      return 0;
   }
   
   // USART2 DMA1 Init
   __HAL_RCC_DMA1_CLK_ENABLE();
	
   // USART1 DMA RX Init
   HDMA_BUS_rx.Instance                   = DMA1_Stream5;
   HDMA_BUS_rx.Init.Channel               = DMA_CHANNEL_4;
   HDMA_BUS_rx.Init.Direction             = DMA_PERIPH_TO_MEMORY;
   HDMA_BUS_rx.Init.PeriphInc             = DMA_PINC_DISABLE;
   HDMA_BUS_rx.Init.MemInc                = DMA_MINC_ENABLE;
   HDMA_BUS_rx.Init.PeriphDataAlignment   = DMA_PDATAALIGN_BYTE;
   HDMA_BUS_rx.Init.MemDataAlignment      = DMA_MDATAALIGN_BYTE;
   HDMA_BUS_rx.Init.Mode                  = DMA_NORMAL;
   HDMA_BUS_rx.Init.Priority              = DMA_PRIORITY_LOW;
   HDMA_BUS_rx.Init.FIFOMode              = DMA_FIFOMODE_DISABLE;
   
   if (HAL_DMA_Init(&HDMA_BUS_rx) != HAL_OK)
   {
      return 0;
   }

    __HAL_LINKDMA(&UART_Bus_Handle,hdmarx,HDMA_BUS_rx);

   // USART1 DMA TX Init
   HDMA_BUS_tx.Instance                   = DMA1_Stream6;
   HDMA_BUS_tx.Init.Channel               = DMA_CHANNEL_4;
   HDMA_BUS_tx.Init.Direction             = DMA_MEMORY_TO_PERIPH;
   HDMA_BUS_tx.Init.PeriphInc             = DMA_PINC_DISABLE;
   HDMA_BUS_tx.Init.MemInc                = DMA_MINC_ENABLE;
   HDMA_BUS_tx.Init.PeriphDataAlignment   = DMA_PDATAALIGN_BYTE;
   HDMA_BUS_tx.Init.MemDataAlignment      = DMA_MDATAALIGN_BYTE;
   HDMA_BUS_tx.Init.Mode                  = DMA_NORMAL;
   HDMA_BUS_tx.Init.Priority              = DMA_PRIORITY_LOW;
   HDMA_BUS_tx.Init.FIFOMode              = DMA_FIFOMODE_DISABLE;

   if (HAL_DMA_Init(&HDMA_BUS_tx) != HAL_OK)
   {
      return 0;
   }

   __HAL_LINKDMA(&UART_Bus_Handle,hdmatx,HDMA_BUS_tx);
   
   // Timer Init for bus/eth access used as framegap timeout timer
   __HAL_RCC_TIM7_CLK_ENABLE();
   TIM7_Handle.Instance                   = TIM7;
   TIM7_Handle.Init.Period                = FRAMEGAPTIME;
   TIM7_Handle.Init.Prescaler             = (uint32_t)((HAL_RCC_GetPCLK2Freq() / 10000000U) - 1U);  // 10 MHz (0.1 us ticks)
   TIM7_Handle.Init.ClockDivision         = 0;
   TIM7_Handle.Init.CounterMode           = TIM_COUNTERMODE_UP;
   TIM7_Handle.Init.AutoReloadPreload     = TIM_AUTORELOAD_PRELOAD_DISABLE;
   HAL_TIM_Base_Init(&TIM7_Handle);
   
   // clear it pending bit to avoid event at beginning
   __HAL_TIM_CLEAR_IT(&TIM7_Handle, TIM_IT_UPDATE);
   
   // Timer Init for bus/eth access used as random timeout timer
   __HAL_RCC_TIM3_CLK_ENABLE();
   TIM3_Handle.Instance                   = TIM3;
   TIM3_Handle.Init.Period                = 0;// 0.1 us
   TIM3_Handle.Init.Prescaler             = (uint32_t)((HAL_RCC_GetPCLK2Freq() / 10000000U) - 1U);  // 10 MHz (0.1 us ticks)
   TIM3_Handle.Init.ClockDivision         = 0;
   TIM3_Handle.Init.CounterMode           = TIM_COUNTERMODE_UP;
   TIM3_Handle.Init.AutoReloadPreload     = TIM_AUTORELOAD_PRELOAD_DISABLE;
   HAL_TIM_Base_Init(&TIM3_Handle);
   
   // clear it pending bit to avoid event at beginning
   __HAL_TIM_CLEAR_IT(&TIM3_Handle, TIM_IT_UPDATE);
   
   // Init NVIC for UART DMA
   // UART
   HAL_NVIC_SetPriority(USART2_IRQn, 7, 0);
   HAL_NVIC_EnableIRQ(USART2_IRQn);
   // DMA TX
   HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 7, 0);
   HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
   // DMA RX
   HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 7, 0);
   HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
   // TIM
   HAL_NVIC_SetPriority(TIM7_IRQn, 8, 0);
   HAL_NVIC_EnableIRQ(TIM7_IRQn);
   HAL_NVIC_SetPriority(TIM3_IRQn, 8, 0);
   HAL_NVIC_EnableIRQ(TIM3_IRQn);
   
   return 1;
}

//------------------------------------------------------------------------------
/// \brief     Function to set RTS/CTS pins on rs485 driver                  
///
/// \param     [in]  uart_cmd_t setter
///
/// \return    none
void bus_uart_setRs485( UART_CMD_t setter )
{
   if( setter != RECEIVE )
   {
      HAL_GPIO_WritePin(UART_GPIO, UART_PIN_BUS_RTS|UART_PIN_BUS_CTS, GPIO_PIN_SET);
   }
   else
   {
      HAL_GPIO_WritePin(UART_GPIO, UART_PIN_BUS_RTS|UART_PIN_BUS_CTS, GPIO_PIN_RESET);
   }   
}

//------------------------------------------------------------------------------
/// \brief     Function returns handler of the bus peripheral             
///
/// \param     -
///
/// \return    none
UART_HandleTypeDef* bus_uart_getHandler( void )
{
   return &UART_Bus_Handle;
}

//------------------------------------------------------------------------------
/// \brief     Function used to send data over uart           
///
/// \param     -
///
/// \return    none
uint8_t bus_uart_send( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t size )
{
   static   uint16_t   uart_tx_err_counter;
            uint32_t   crc32;
            uint8_t*   crcFragment;
            uint32_t   tickstart;
   static   uint32_t   randtimeoutError;
   static   uint32_t   uartRdyError;
   static   uint32_t   gaptimeoutError;
   static   uint32_t   busIdleError;
   static   uint32_t   randomcounter;
   
   // time to calculate crc32 in hardware
   crc32 = bus_uart_calcCRC( (uint32_t*)&pData[MACDSTFIELD], (uint32_t)size );
   
   // append crc to the outputbuffer
   crcFragment = (uint8_t*)&crc32;
   
   for( uint8_t i=0, j=3; i<4; i++,j-- )
   {
      *(pData+MACDSTFIELD+size+i) = *(crcFragment+j);
   }
   
   // wait for the peripheral to be ready
   tickstart = HAL_GetTick();
   while( huart->gState != HAL_UART_STATE_READY )
   {
      if((HAL_GetTick() - tickstart) > 500u)  // 1u = 1 ms
      {
         uartRdyError++;
         return 0;
      }
   }
   
   // if necessary, wait for interframegap end
   while( framegapTimeoutFlag != SET );
   
   // start the random countdown to check if the bus is not occupied
   do
   {
      bus_uart_startRandomTimeoutTx();
      while( randomTimeoutFlag != SET );
   }while( framegapTimeoutFlag != SET );
   
   // switch the RS485 transceiver into transmit mode
   bus_uart_setRs485( TRANSMIT );
   
   // start transmitting in interrupt mode
   __HAL_UART_DISABLE_IT(&UART_Bus_Handle, UART_IT_IDLE);         // disable idle line interrupt
   __HAL_UART_DISABLE_IT(&UART_Bus_Handle, UART_IT_RXNE);         // disable rx interrupt
          
   // send the data
   if( HAL_UART_Transmit_DMA(huart, pData, size+(uint16_t)PREAMBLESFDLENGTH+(uint16_t)CRC32LENGTH ) != HAL_OK )
   {
      uart_tx_err_counter++;
      return 0;
   }
   
   return 1;
}

//------------------------------------------------------------------------------
/// \brief     Function to receive data over uart
///
/// \param     -
///
/// \return    none
inline void bus_uart_receive( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t size )
{
   static uint16_t uart_rx_err_counter;
   
   // wait for the peripheral to be ready
   while(UART_Bus_Handle.gState != HAL_UART_STATE_READY);
   
   // switch the RS485 transceiver into receive mode
   bus_uart_setRs485( RECEIVE );
   
   // enable idle line and rx interrupt
   __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
   __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
   
   // start receiving in interrupt mode
   if(HAL_UART_Receive_DMA(huart, pData, size) != HAL_OK)
   {
      uart_rx_err_counter++;
   }
}

//------------------------------------------------------------------------------
/// \brief     Tx Transfer completed callback                   
///
/// \param     - 
///
/// \return    none
void bus_uart_TxCpltCallback( void )
{
   // stop uart
   HAL_UART_DMAStop(&UART_Bus_Handle);
   HAL_UART_Abort_IT(&UART_Bus_Handle);
   
   // notify the tcp/ip stack
   bus_uart_transferCompleteTasknotify();
   
   // start receiving with new flags
   bus_uart_receive( &UART_Bus_Handle, busuartRx.buffer, busuartRx.bufferSize );
}

//------------------------------------------------------------------------------
/// \brief     Rx Transfer completed callback (not needed, because idle line
///            detection is being used.                   
///
/// \param     - 
///
/// \return    none
void bus_uart_RxCpltCallback( void )
{
   static uint16_t rxCpltCounter;
   rxCpltCounter++;
}

//------------------------------------------------------------------------------
/// \brief     Error callback of the uart peripheral                   
///
/// \param     - 
///
/// \return    none
void bus_uart_ErrorCallback( void )
{
   static uint16_t errorCallbackCounter;
   errorCallbackCounter++;
}

//------------------------------------------------------------------------------
/// \brief     Error abort callback of the uart peripheral                   
///
/// \param     - 
///
/// \return    none
void bus_uart_AbortCallback( void )
{
   static uint16_t abortCallbackCounter;
   abortCallbackCounter++;
}

//------------------------------------------------------------------------------
/// \brief     Rx idle line detection callback                   
///
/// \param     - 
///
/// \return    none
inline void bus_uart_IdleLnCallback( void )
{
   // stop irq and reset flags and values
   HAL_UART_DMAStop(&UART_Bus_Handle);
   HAL_UART_Abort_IT(&UART_Bus_Handle);
   __HAL_UART_DISABLE_IT(&UART_Bus_Handle, UART_IT_IDLE);         // disable idle line interrupt
   __HAL_UART_DISABLE_IT(&UART_Bus_Handle, UART_IT_RXNE);         // disable rx interrupt
   
   // get message length
   busuartRx.frameSize = busuartRx.bufferSize - (uint16_t)(HDMA_BUS_rx.Instance->NDTR);

   // abort if input data is 0 bytes in length or too long or too short for a ethernet frame
   if( busuartRx.frameSize == 1 )
   {
      // increment error counter
      bus_uart_statistic.counterFrame1Byte++;
      
      // start receiving with new flags
      bus_uart_receive( &UART_Bus_Handle, busuartRx.buffer, busuartRx.bufferSize );
      return;
   }
   if( busuartRx.frameSize == 0 || busuartRx.frameSize < MINSIZE )
   {
      // increment error counter
      bus_uart_statistic.counterFrameTooShort++;
      
      // start receiving with new flags
      bus_uart_receive( &UART_Bus_Handle, busuartRx.buffer, busuartRx.bufferSize );
      return;
   }
   if( busuartRx.frameSize > ETHSIZE+PREAMBLESFDLENGTH )
   {
      // increment error counter
      bus_uart_statistic.counterFrameTooLong++;
      
      // start receiving with new flags
      bus_uart_receive( &UART_Bus_Handle, busuartRx.buffer, busuartRx.bufferSize );
      return;
   }

   // check for the preamble and start frame delimiter
   if(( memcmp( ( void * ) (busuartRx.buffer+6), ( void * ) (preAmbleSFD+6), (PREAMBLESFDLENGTH-6)) != 0 ))
   {
      // increment error counter
      bus_uart_statistic.counterFrameInvalidPreamble++;
      // start receiving with new flags
      //bus_uart_receive( &UART_Bus_Handle, busuartRx.buffer, busuartRx.bufferSize );
      //return;
   }
   else
   {
      busEthFrameDetected = SET;
   }
   
   // mac address filter
   if( bus_uart_macCheck( &busuartRx.buffer[8] ) != 1 )
   {
      // increment error counter
      bus_uart_statistic.counterMACAlienFrame++;
      
      // start receiving with new flags
      bus_uart_receive( &UART_Bus_Handle, busuartRx.buffer, busuartRx.bufferSize );
      return;
   }

   // ARP Filter
   if( memcmp( &busuartRx.buffer[20], arpCode, 2 ) == 0 )
   {
      // is the arp determined for my ip address?
      if( bus_uart_ipCheck( &busuartRx.buffer[46] ) != 1 )
      {
         // increment error counter
         bus_uart_statistic.counterARPAlienFrame++;
         
         // start receiving with new flags
         bus_uart_receive( &UART_Bus_Handle, busuartRx.buffer, busuartRx.bufferSize );
         return;
      }
   }
   
   // check the frames crc
   if( HAL_CRC_Calculate(&CRC_Handle, (uint32_t*)(busuartRx.buffer+MACDSTFIELD), (uint32_t)(busuartRx.frameSize-PREAMBLESFDLENGTH)) != 0 )
   {
      // increment error counter
      bus_uart_statistic.counterFrameInvalidCRC++;
      
      // start receiving with new flags
      bus_uart_receive( &UART_Bus_Handle, busuartRx.buffer, busuartRx.bufferSize );
      return;
   }
   
   // integer type wrap arround check and set for head and tail index
   if( inputFrameHandle.inputFrameHeadIndex < INPUTFRAMEFIFOSIZE )
   {
      inputFrameHandle.inputFrameHeadIndex += INPUTFRAMEFIFOSIZE;
      inputFrameHandle.inputFrameTailIndex += INPUTFRAMEFIFOSIZE;
   }

   // head index needs to be always ahead of tail index, head=tail index means the fifo is empty
   if( inputFrameHandle.inputFrameHeadIndex - inputFrameHandle.inputFrameTailIndex < INPUTFRAMEFIFOSIZE )
   {
      // the flag to valid, so this slot on the fifo is ready to be computed
      uint8_t arrayIndex = inputFrameHandle.inputFrameHeadIndex%INPUTFRAMEFIFOSIZE;
      
      // store the framesize
      inputFrameHandle.inputFrameFifo[arrayIndex].frameSize = (uint16_t)(busuartRx.frameSize-PREAMBLESFDLENGTH-CRC32LENGTH);
      
      // set the pointer to the framestart
      inputFrameHandle.inputFrameFifo[arrayIndex].frameStart = busuartRx.buffer+MACDSTFIELD;

      // set the head index of the fifo
      busuartRx.buffer = inputFrameHandle.inputFrameFifo[inputFrameHandle.inputFrameHeadIndex%INPUTFRAMEFIFOSIZE].frameArray;
      
      // increment fifo frame counter
      inputFrameHandle.inputFrameCounter++;
      
      // increment head counter
      inputFrameHandle.inputFrameHeadIndex++;
      
      // increment the fifo
      inputFrameHandle.inputFrameCurrentFifoSize++;
      
      // set the peak if necessary
      if( inputFrameHandle.inputFrameCurrentFifoSize > inputFrameHandle.inputFramePeakFifoSize )
      {
         inputFrameHandle.inputFramePeakFifoSize = inputFrameHandle.inputFrameCurrentFifoSize;
      }
   }
   else
   {
      // increment error counter
      bus_uart_statistic.counterFifoFull++;
      
      // start receiving with new flags
      bus_uart_receive( &UART_Bus_Handle, busuartRx.buffer, busuartRx.bufferSize );
      return;
   }
     
   // increment the counter for successful received frames
   bus_uart_statistic.counterValidFrame++;
   
   // set flags and values to start receiving conditions
   bus_uart_receive( &UART_Bus_Handle, busuartRx.buffer, busuartRx.bufferSize );
}

//------------------------------------------------------------------------------
/// \brief     If the fifo has a frame to process, then the function returns set      
///
/// \param     - 
///
/// \return    Set = Frame to process, Reset = fifo is empty
FlagStatus bus_uart_getFifoFlag( void )
{
   //static uint32_t getFifoFlagError;
   
   if( inputFrameHandle.inputFrameHeadIndex > inputFrameHandle.inputFrameTailIndex )
   {
      return SET;
   }

   return RESET;
}

//------------------------------------------------------------------------------
/// \brief     The function returns a pointer to a frame object
///
/// \param     -
///
/// \return    pointer to frame object, if NULL then there is no frame in the fifo
BUS_UART_INPUT_ETHFRAME_t* bus_uart_getFrameFromFifo( void )
{
   static uint32_t fifoSizeError;
   
   // return the oldest frame from the fifo
   if( inputFrameHandle.inputFrameHeadIndex > inputFrameHandle.inputFrameTailIndex )
   {
      return &inputFrameHandle.inputFrameFifo[inputFrameHandle.inputFrameTailIndex%INPUTFRAMEFIFOSIZE];
   }
   else if( inputFrameHandle.inputFrameHeadIndex == inputFrameHandle.inputFrameTailIndex )
   {
      // fifo is empty
      return NULL;
   }
   else
   {
      fifoSizeError++;
   }
   return NULL;
}

//------------------------------------------------------------------------------
/// \brief     Acknowledge the last taken frame from the fifo
///
/// \param     none
///
/// \return    none
void bus_uart_ackFrameFromFifo( void )
{
   static uint32_t fifoAckError;
   static uint32_t fifoSizeErrorZero;

   // check fifo size
   if( inputFrameHandle.inputFrameHeadIndex > inputFrameHandle.inputFrameTailIndex )
   {
      // check for fifo error
      if( inputFrameHandle.inputFrameCurrentFifoSize == 0 )
      {
         // increment error counter
         fifoSizeErrorZero++;
         
         // correct the fifosize
         inputFrameHandle.inputFrameCurrentFifoSize = 1;
      }
      
      // decrement fifo frame counter
      inputFrameHandle.inputFrameCurrentFifoSize--;
      
      // increment tail counter
      inputFrameHandle.inputFrameTailIndex++;
   }
   else
   {
      fifoAckError++;
   }
}

//------------------------------------------------------------------------------
/// \brief     Returns size of the last received ethernet frame                 
///
/// \param     - 
///
/// \return    uint16_t size
uint16_t bus_uart_getSize( void )
{
   return busuartRx.frameSize;
}

//------------------------------------------------------------------------------
/// \brief     Returns pointer to the rx buffer          
///
/// \param     - 
///
/// \return    pointer to uart rx descriptor busuartRx.buffer
uint8_t* bus_uart_getRxBufferpointer( void )
{
   return busuartRx.buffer;
}

//------------------------------------------------------------------------------
/// \brief     Returns pointer to the tx buffer       
///
/// \param     - 
///
/// \return    pointer to txBuffer1
uint8_t* bus_uart_getTxBufferpointer( void )
{
   return txBuffer1;
}

//------------------------------------------------------------------------------
/// \brief     Returns pointer to the buffer length              
///
/// \param     - 
///
/// \return    uint16_t busuartRx.buffer
uint16_t bus_uart_getBuffersize( void )
{
   return busuartRx.bufferSize;
}

//-----------------------------------------------------------------------------
/// \brief      This function resets rx dependend flags and 
///             variables
///
/// \return     
void bus_uart_resetRx( void )
{
   // reset flags and variables
   busuartRx.uartState = 0;
   // reset command information
   busuartRx.frameSize = 0;
}

//------------------------------------------------------------------------------
/// \brief     Calculates CRC Value of given data and length             
///
/// \param     [in] data pointer 
///            [in] data length
///
/// \return    checksum value
uint32_t bus_uart_calcCRC( uint32_t* dataPointer, uint32_t dataLength )
{
   return HAL_CRC_Calculate(&CRC_Handle, dataPointer, dataLength);
}

//------------------------------------------------------------------------------
/// \brief     Check for Mac Address
///
/// \param     [in] pointer to mac address
///
/// \return    1 = address ok, 0 = address not for this device
inline static uint8_t bus_uart_macCheck( uint8_t* pointerToMacAddress )
{
   if( memcmp( pointerToMacAddress, stackMacAddress, 6 ) == 0 )
   {
      return 1;
   }
   else if( memcmp( pointerToMacAddress, macBroadcast, 6 ) == 0 )
   {
      bus_uart_statistic.counterMACBroadcast++;
      return 1;
   }
   else if( memcmp( pointerToMacAddress, macLLMNR, 6 ) == 0 )
   {
      bus_uart_statistic.counterMACLLMNR++;
      return 1;
   }
   else
   {
      return 0;
   }
}

//------------------------------------------------------------------------------
/// \brief     Check for ip Address
///
/// \param     [in] pointer to ip address
///
/// \return    1 = address ok, 0 = address not for this device
static uint8_t bus_uart_ipCheck( uint8_t* pointerToIpAddress )
{
   uint32_t    ipAddress;
   uint8_t*    ipAddress8b;
   
   // get ip configuration
   FreeRTOS_GetAddressConfiguration( &ipAddress, NULL, NULL, NULL );
   ipAddress8b = (uint8_t*)(&ipAddress);
     
   if( memcmp( pointerToIpAddress, ipAddress8b, 4 ) == 0 )
   {
      return 1;
   }
   else
   {
      return 0;
   }
}

//------------------------------------------------------------------------------
/// \brief     Returns the uart descriptor
///
/// \param     [in] pointer to current bus descriptor
///
/// \return    none
BUS_UART_RX_t* bus_uart_getUartDescriptor( void )
{
   return &busuartRx;
}

//------------------------------------------------------------------------------
/// \brief     Callback function for timer 3
///
/// \param     -
///
/// \return    none
inline static void bus_uart_startRandomTimeoutTx( void )
{
   // reset timeout flag
   randomTimeoutFlag = RESET;
   // set a random number for the auto reload register
   TIM3->ARR = (uint32_t)(bus_uart_tcp_getRandomNumber() % 100)*10+100;
   // set counter value to 0
   TIM3->CNT = 0;
   // start the timer
   HAL_TIM_Base_Start_IT(&TIM3_Handle);
}

//------------------------------------------------------------------------------
/// \brief     Set and start timeout function ~300 ns excecution time
///
/// \param     -
///
/// \return    none
inline void bus_uart_startFramegap( void )
{
   // reset timeout flag
   framegapTimeoutFlag = RESET;
  __HAL_TIM_DISABLE_IT(&TIM7_Handle, TIM_IT_UPDATE);
   TIM7->CNT = 0;
   __HAL_TIM_ENABLE_IT(&TIM7_Handle, TIM_IT_UPDATE);
   __HAL_TIM_ENABLE(&TIM7_Handle);
}

FlagStatus bus_uart_getTimeoutFlag( void )
{
   return framegapTimeoutFlag;
}

//------------------------------------------------------------------------------
/// \brief     Callback function for timer 7
///
/// \param     -
///
/// \return    none
void bus_uart_framegapTimeoutCallback( void )
{
   // stop the timer
   HAL_TIM_Base_Stop_IT(&TIM7_Handle);
   __HAL_TIM_CLEAR_IT(&TIM7_Handle, TIM_IT_UPDATE);
   
   // set the timeout flag to 1
   framegapTimeoutFlag = SET;
}

//------------------------------------------------------------------------------
/// \brief     Callback function for timer 3, which sets random timeout flag
///
/// \param     -
///
/// \return    none
inline void bus_uart_randomTimeoutCallback( void )
{
   // stop the timer
   HAL_TIM_Base_Stop_IT(&TIM3_Handle);
   __HAL_TIM_CLEAR_IT(&TIM3_Handle, TIM_IT_UPDATE);
   
   // set the timeout flag to 1
   randomTimeoutFlag = SET;
}

//------------------------------------------------------------------------------
/// \brief     Function to test the CRC peripheral
///
/// \param     -
///
/// \return    none
static void bus_uart_testCRC( void )
{
   // crc test
   // create test array
   volatile uint32_t crc32_1, crc32_2, crc32_3, crc32_4, crc32_5, crc32_6, crc32_7;
   const uint8_t testArray[] = { 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x20 };
   static volatile uint8_t results[6];
   const uint32_t crc32_9bytes = 0x3489D553;
   const uint32_t crc32_8bytes = 0xAF65CA88;
   const uint32_t crc32_7bytes = 0xC295B54A;
   const uint32_t crc32_6bytes = 0xD5AED2A3;
   const uint32_t crc32_5bytes = 0x0AA8728F;
   const uint32_t crc32_4bytes = 0x9D48EF59;
   volatile uint8_t crc32_9bytesOKFlag  = 0;
   volatile uint8_t crc32_8bytesOKFlag  = 0;
   volatile uint8_t crc32_7bytesOKFlag  = 0;
   volatile uint8_t crc32_6bytesOKFlag  = 0;
   volatile uint8_t crc32_5bytesOKFlag  = 0;
   volatile uint8_t crc32_4bytesOKFlag  = 0;

   crc32_7 = bus_uart_calcCRC( (uint32_t*)&testArray[0], (uint32_t)9 );
   if( (uint32_t)crc32_9bytes == (uint32_t)crc32_7 )
   {
      crc32_9bytesOKFlag = 1;
   }
   else
   {
      crc32_9bytesOKFlag = 0;
   }
   results[0] = crc32_9bytesOKFlag;
   
   crc32_1 = bus_uart_calcCRC( (uint32_t*)&testArray[0], (uint32_t)8 );
   if( (uint32_t)crc32_8bytes == (uint32_t)crc32_1 )
   {
      crc32_8bytesOKFlag = 1;
   }
   else
   {
      crc32_8bytesOKFlag = 0;
   }
   results[1] = crc32_8bytesOKFlag;
   
   crc32_2 = bus_uart_calcCRC( (uint32_t*)&testArray[0], (uint32_t)7 );
   if( crc32_7bytes == crc32_2 )
   {
      crc32_7bytesOKFlag = 1;
   }
   else
   {
      crc32_7bytesOKFlag = 0;
   }
   results[2] = crc32_7bytesOKFlag;
   
   crc32_3 = bus_uart_calcCRC( (uint32_t*)&testArray[0], (uint32_t)6 );
   if( crc32_6bytes == crc32_3 )
   {
      crc32_6bytesOKFlag = 1;
   }
   else
   {
      crc32_6bytesOKFlag = 0;
   }
   results[3] = crc32_6bytesOKFlag;
   
   crc32_4 = bus_uart_calcCRC( (uint32_t*)&testArray[0], (uint32_t)5 );
   if( crc32_5bytes == crc32_4 )
   {
      crc32_5bytesOKFlag = 1;
   }
   else
   {
      crc32_5bytesOKFlag = 0;
   }
   results[4] = crc32_5bytesOKFlag;
   
   crc32_5 = bus_uart_calcCRC( (uint32_t*)&testArray[0], (uint32_t)4 );
   if( crc32_4bytes == crc32_5 )
   {
      crc32_4bytesOKFlag = 1;
   }
   else
   {
      crc32_4bytesOKFlag = 0;
   }
   results[5] = crc32_4bytesOKFlag;
}

uint8_t bus_uart_frameCheck( uint8_t* framePointer, uint16_t frameLength )
{  
   // check for the preamble and start frame delimiter
   if(( memcmp( ( void * ) framePointer, ( void * ) preAmbleSFD, PREAMBLESFDLENGTH) != 0 ))
   {
      return 0;
   }
   
   // check the frames crc
   if( HAL_CRC_Calculate(&CRC_Handle, (uint32_t*)(framePointer+MACDSTFIELD), (uint32_t)(frameLength-PREAMBLESFDLENGTH) ) != 0 )
   {
      return 0;
   }
 
   // both preamble/sfd and crc check has been successful
   return 1;
}

//------------------------------------------------------------------------------
/// \brief     Return the pointer to the mac frame
///
/// \param     -
///
/// \return    pointer to mac frame
uint8_t* bus_uart_getMacFramePointer( void )
{
   return macFramePointer;
}

//------------------------------------------------------------------------------
/// \brief     Returns the length of the mac frame
///
/// \param     -
///
/// \return    mac frame length
uint16_t bus_uart_getMacFrameLength( void )
{
   return macFrameLength;
}

//------------------------------------------------------------------------------
/// \brief     Return flag status of the valid ethernet frame received flag
///
/// \param     -
///
/// \return    flag status
uint8_t bus_uart_getValidEthFrameFlag( void )
{
   return busEthFrameDetected;
}

//------------------------------------------------------------------------------
/// \brief     Disables all bus irqs
///
/// \param     -
///
/// \return    none status
void bus_uart_disableIRQ( void )
{
   // UART
   HAL_NVIC_DisableIRQ(USART2_IRQn);
   // DMA TX
   HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
   // DMA RX
   HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
   // TIM
   HAL_NVIC_DisableIRQ(TIM7_IRQn);
   HAL_NVIC_DisableIRQ(TIM3_IRQn);
}

//------------------------------------------------------------------------------
/// \brief     Enables all bus irqs
///
/// \param     -
///
/// \return    none status
void bus_uart_enableIRQ( void )
{
   // UART
   HAL_NVIC_EnableIRQ(USART2_IRQn);
   // DMA TX
   HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
   // DMA RX
   HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
   // TIM
   HAL_NVIC_EnableIRQ(TIM7_IRQn);
   HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

//------------------------------------------------------------------------------
/// \brief     Returns bus statistics pointer
///
/// \param     -
///
/// \return    pointer to the bus statistics
BUS_UART_STATISTIC_t* bus_uart_getStatistics( void )
{
   return &bus_uart_statistic;
}