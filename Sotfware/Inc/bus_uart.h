// ****************************************************************************
/// \file      bus_uart.h
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

// Define to prevent recursive inclusion **************************************
#ifndef __BUS_UART_H
#define __BUS_UART_H

// Include ********************************************************************
#include "stm32f4xx_hal.h"

// Exported defines ***********************************************************
// UART bus communication
#define UART_PIN_BUS_RX                GPIO_PIN_6
#define UART_PIN_BUS_TX                GPIO_PIN_5
#define UART_PIN_BUS_RTS               GPIO_PIN_4
#define UART_PIN_BUS_CTS               GPIO_PIN_3
#define UART_PIN_BUS_BE                GPIO_PIN_2
#define UART_GPIO                      GPIOD
#define UART_BUS_BAUDRATE              ( 6000000u )

// Ethernet
#define MACDSTFIELD              ( 8 )
#define TOTALLENGTHLSBFIELD      ( 17 )
#define TOTALLENGTHMSBFIELD      ( 16 )
#define MACSRCADRLENGTH          ( 6 )
#define MACDSTADRLENGTH          ( 6 )
#define ETHTYPELENGTH            ( 2 )
#define PREAMBLESFDLENGTH        ( 8 )
#define CRC32LENGTH              ( 4 )
#define ETHSIZE                  ( 1548 )    // The original Ethernet IEEE 802.3 
                                             // standard defined the minimum 
                                             // Ethernet frame size as 64 bytes 
                                             // and the maximum as 1518 bytes. 
                                             // The maximum was later increased 
                                             // to 1522 bytes to allow for VLAN 
                                             // tagging. The minimum size of an 
                                             // Ethernet frame that carries an 
                                             // ICMP packet is 74 bytes.
                                             // +8 bytes for preamble & sfd
#define MINSIZE                  ( PREAMBLESFDLENGTH+CRC32LENGTH )
#define TYPEARP                  ( 0806 )
#define ETHTYPEFIELDMSB          ( 12 )
#define ETHTYPEFIELDLSB          ( 13 )
#define MINPAYLOAD               ( 46 )
#define INPUTFRAMEFIFOSIZE       ( 10 )

// Exported types *************************************************************
typedef struct BUS_UART_STATISTIC_s
{
   uint32_t                      counterFrame1Byte;   
   uint32_t                      counterFrameTooLong;   
   uint32_t                      counterFrameTooShort;              
   uint32_t                      counterMACAlienFrame;  
   uint32_t                      counterMACBroadcast;  
   uint32_t                      counterMACLLMNR;  
   uint32_t                      counterARPAlienFrame;                       
   uint32_t                      counterFrameInvalidPreamble;
   uint32_t                      counterFrameInvalidCRC;
   uint32_t                      counterFifoFull;
   uint32_t                      counterValidFrame;
}BUS_UART_STATISTIC_t;

// UART
typedef enum UART_RETURN_e
{
   UART_OK,                ///< OK
   UART_ERROR              ///< unspecified error
} UART_RETURN_t;

// UART BUS
typedef enum
{
   RECEIVE = 0U,
   TRANSMIT             
} UART_CMD_t;

typedef enum 
{
  CHANGEDDATA = 0U, 
  ALLDATA = !CHANGEDDATA
} OptionFlag;

typedef struct BUS_UART_RX_s
{
   uint8_t        uartState;                       ///< state of the uart communication    0 = idle, 1 = busy           
   uint8_t*       buffer;                          ///< pointer to buffer
   uint16_t       bufferSize;                      ///< size to the pointed buffer
   uint16_t       frameSize;                       ///< size of the complete eth frame
   uint16_t       byteCounter;                     ///< size of the data field in the message                             
} BUS_UART_RX_t;

typedef struct BUS_UART_INPUT_ETHFRAME_s
{
   uint8_t        frameArray[ETHSIZE];             ///< a buffer with enough size to hold a maximum sized ethernet frame
   uint8_t*       frameStart;                      ///< start of the frame without preamble and sfd  
   uint16_t       frameSize;                       ///< size of the stored frame                          
} BUS_UART_INPUT_ETHFRAME_t;

typedef struct BUS_UART_INPUT_FIFO_HANDL_s
{
   uint8_t                    inputFrameCurrentFifoSize;             ///< the counter to show how many valid frames are in the fifo
   uint32_t                   inputFramePeakFifoSize;                ///< shows the peak of the fifo
   uint32_t                   inputFrameCounter;                     ///< counter for how many frames have been stored in the fifo so far
   uint32_t                   inputFrameCounterProcessed;            ///< counter for how many frames have been stored in the fifo so far and processed
   uint32_t                   inputFrameFifoFullFlag;                ///< a flag to show if the fifo is full or still capable to store frames 
   uint32_t                   inputFrameFifoFullCounter;             ///< counts how many times the fifo was full
   uint32_t                   inputFrameHeadIndex;                   ///< head index of the fifo
   uint32_t                   inputFrameTailIndex;                   ///< tail index of the fifo
   uint32_t                   inputFrameProcessingNr;                ///< array index number of the computed frame
   uint16_t                   inputFrameFifoCorrupted;               ///< if the processing number differs from the tailnumber, the fifo is broken
   BUS_UART_INPUT_ETHFRAME_t  inputFrameFifo[INPUTFRAMEFIFOSIZE];    ///< pointer to the fifo               
} BUS_UART_INPUT_FIFO_HANDLE_t;

// Exported functions *********************************************************
uint8_t                    bus_uart_init                    ( void );
uint8_t                    bus_uart_deinit                  ( void );
void                       bus_uart_setRs485                ( UART_CMD_t setter );
UART_HandleTypeDef*        bus_uart_getHandler              ( void );
uint16_t                   bus_uart_getSize                 ( void );
uint8_t*                   bus_uart_getRxBufferpointer      ( void );
uint8_t*                   bus_uart_getTxBufferpointer      ( void );
uint16_t                   bus_uart_getBuffersize           ( void );
void                       bus_uart_resetRx                 ( void );
uint8_t                    bus_uart_send                    ( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size );
void                       bus_uart_receive                 ( UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size );
void                       bus_uart_TxCpltCallback          ( void );
void                       bus_uart_RxCpltCallback          ( void );
void                       bus_uart_IdleLnCallback          ( void );
void                       bus_uart_AbortCallback           ( void );
void                       bus_uart_ErrorCallback           ( void );
uint32_t                   bus_uart_calcCRC                 ( uint32_t* dataPointer, uint32_t dataLength );
BUS_UART_RX_t*             bus_uart_getUartDescriptor       ( void );
void                       bus_uart_framegapTimeoutCallback ( void );
void                       bus_uart_randomTimeoutCallback   ( void );
uint8_t*                   bus_uart_getRxBufferToRead       ( void );
uint16_t                   bus_uart_getSizeToRead           ( void );
uint8_t                    bus_uart_frameCheck              ( uint8_t* framePointer, uint16_t frameLength );
uint8_t*                   bus_uart_getMacFramePointer      ( void );
uint16_t                   bus_uart_getMacFrameLength       ( void );
BUS_UART_INPUT_ETHFRAME_t* bus_uart_getFrameFromFifo        ( void );
void                       bus_uart_ackFrameFromFifo        ( void );
FlagStatus                 bus_uart_getTimeoutFlag          ( void );
uint8_t                    bus_uart_getValidEthFrameFlag    ( void );
void                       bus_uart_disableIRQ              ( void );
void                       bus_uart_enableIRQ               ( void );
BUS_UART_STATISTIC_t*      bus_uart_getStatistics           ( void );
void                       bus_uart_startFramegap           ( void );
FlagStatus                 bus_uart_getTxCpltFlag           ( void );
FlagStatus                 bus_uart_getFifoFlag             ( void );
#endif // __BUS_UART_H