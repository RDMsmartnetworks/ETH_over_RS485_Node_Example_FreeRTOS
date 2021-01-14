// ****************************************************************************
/// \file      bus_uart_tcp.h
///
/// \brief     TCP IP stack entry Module
///
/// \details   Module for the handling of the freertos tcp ip stack
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
#ifndef __BUS_UART_TCP_H
#define __BUS_UART_TCP_H

// Include ********************************************************************
#include "stm32f4xx_hal.h"
#include <FreeRTOS.h>
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

// Exported defines ***********************************************************

// Exported types *************************************************************
typedef enum 
{
  IP_DHCP, 
  IP_STATIC,
  IP_ERROR
} IpMode_t;

// Exported functions *********************************************************
void                    init_tcp_ip_stack             ( void );
void                    deinit_tcp_ip_stack           ( void );
Socket_t                create_TCP_client_socket      ( void );
void                    send_TCP                      ( char *pcBufferToTransmit, const size_t xTotalLengthToSend );
void                    create_tcp_server             ( uint16_t usStackSize, UBaseType_t uxPriority );
void                    invoke_rx_task                ( void );
void                    start_tx_task                 ( void );
BaseType_t              vSendPing                     ( const int8_t *pcIPAddress );
uint8_t                 bus_uart_allocMemory          ( uint8_t* buffer, size_t length );
const uint8_t*          getMacAddress                 ( void );
uint8_t                 eth_setMAC                    ( const uint8_t* string );
void                    bus_uart_rxTaskStartnotify    ( void );
void                    bus_uart_tcp_rngInit          ( void );
void                    bus_uart_tcp_rngDeInit        ( void );
const char*             pcApplicationHostnameHookCAP  ( void );
uint32_t                bus_uart_tcp_getRandomNumber  ( void );
void                    bus_uart_tcp_refreshHostname  ( void );
#endif // __BUS_UART_TCP_H