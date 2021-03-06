/*
 * Some constants, hardware definitions and comments taken from ST's HAL driver
 * library, COPYRIGHT(c) 2015 STMicroelectronics.
 */

/*
 * FreeRTOS+TCP V2.0.3
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */


// Include ********************************************************************
/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_DNS.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"
#include "phyHandling.h"

/* ST includes. */
#include "stm32f4xx_hal.h"
    
/* Project includes. */
#include "bus_uart.h"

// Private define *************************************************************

// Private types     **********************************************************

// Private variables **********************************************************
static TaskHandle_t        uartTransferCompleteToNotify;      // task notify variable for uart rx task

// Private function prototypes ************************************************

// Global variables     *******************************************************
extern UART_HandleTypeDef  UART_Bus_Handle;
    
BaseType_t xNetworkInterfaceInitialise( void )
{
BaseType_t xReturn;

    /*
     * Perform the hardware specific network initialisation here.  Typically
     * that will involve using the Ethernet driver library to initialise the
     * Ethernet (or other network) hardware, initialise DMA descriptors, and
     * perform a PHY auto-negotiation to obtain a network link.
     *
     * This example assumes InitialiseNetwork() is an Ethernet peripheral driver
     * library function that returns 0 if the initialisation fails.
     */
    if( bus_uart_init() == 0 )
    {
        xReturn = pdFAIL;
    }
    else
    {
        xReturn = pdPASS;
    }

    return xReturn;
}

/* Simple network interfaces (as opposed to more efficient zero copy network
interfaces) just use Ethernet peripheral driver library functions to copy
data from the FreeRTOS+TCP buffer into the peripheral driver's own buffer.
This example assumes SendData() is a peripheral driver library function that
takes a pointer to the start of the data to be sent and the length of the
data to be sent as two separate parameters.  The start of the data is located
by pxDescriptor->pucEthernetBuffer.  The length of the data is located
by pxDescriptor->xDataLength. */
BaseType_t xNetworkInterfaceOutput( NetworkBufferDescriptor_t * const pxDescriptor, BaseType_t xReleaseAfterSend )
{
   static     uint8_t*  txData;
   static     uint16_t  txDataLength;
   static     uint8_t*  txBuffer;
   
   // get the buffer for sending data out
   txBuffer = bus_uart_getTxBufferpointer();
   
   // fix pointer length from the rtos buffer
   txData         = pxDescriptor->pucEthernetBuffer;
   txDataLength   = pxDescriptor->xDataLength;

   // copy data into tx output buffer
   memcpy( &txBuffer[MACDSTFIELD], txData, txDataLength );

   // send the data in the buffer
   bus_uart_send( &UART_Bus_Handle, txBuffer, txDataLength );
   
   // setup a notify
   uartTransferCompleteToNotify = xTaskGetCurrentTaskHandle();
   
   // task will be resumed after notify has been given from the tx complete callback
   if( ulTaskNotifyTake( pdTRUE, 5000 ) == 0 )
   {
      static volatile uint32_t transferCompleteError;
      // no transfer complete interrupt has been triggeret
      transferCompleteError++;
   }

   // start to receive again
   //bus_uart_receive( &UART_Bus_Handle, bus_uart_getRxBufferpointer(), bus_uart_getBuffersize() );
   
   // finish the transmission
   // release the allocated buffer
   if( xReleaseAfterSend != pdFALSE )
   {
      // It is assumed SendData() copies the data out of the FreeRTOS+TCP Ethernet
      // buffer.  The Ethernet buffer is therefore no longer needed, and must be
      // freed for re-use.
      vReleaseNetworkBufferAndDescriptor( pxDescriptor );
   }  
   
   // Call the standard trace macro to log the send event.
   iptraceNETWORK_INTERFACE_TRANSMIT();
   
   return pdTRUE;
}

//-----------------------------------------------------------------------------
/// \brief     Tasknotify callback function from the TX complete irq.
///
/// \param     none
///
/// \return    none
void bus_uart_transferCompleteTasknotify( void )
{
   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
   
   // Wakeup the sblink receive task
   if( uartTransferCompleteToNotify != NULL )
   {
		vTaskNotifyGiveFromISR( uartTransferCompleteToNotify, &xHigherPriorityTaskWoken );
      uartTransferCompleteToNotify = NULL;
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}