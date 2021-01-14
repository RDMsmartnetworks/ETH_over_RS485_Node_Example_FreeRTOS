// ****************************************************************************
/// \file      bus_uart_tcp.c
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

// Include ********************************************************************
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "bus_uart_tcp.h"
#include "bus_uart.h"
#include "http.h"
#include "FreeRTOS_IP_Private.h"
#include "NetworkBufferManagement.h"
#include "main.h"

// Private define *************************************************************

// Private types     **********************************************************

// Global variables ***********************************************************
  
// Private variables **********************************************************
volatile static UBaseType_t ulNextRand;
static TaskHandle_t  rxTaskToNotify                   = NULL;
static TaskHandle_t  rxTaskStartToNotify              = NULL;
static TaskHandle_t  serverTask                       = NULL;
static TaskHandle_t  clientTask                       = NULL;
static char*         mainHOST_NAME                    = "example_device";
static char*         mainDEVICE_NICK_NAME             = "example_device";
static char*         mainHOST_NAMEcapLetters          = "EXAMPLE_DEVICE";
static char*         mainDEVICE_NICK_NAMEcapLetters   = "EXAMPLE_DEVICE";
static FlagStatus    processingIdle = SET;
// default mac and ip address in the flash
const uint8_t        ucIPAddressFLASH[4]              = { 10, 0, 0, 111 };
const uint8_t        ucNetMaskFLASH[4]                = { 255, 255, 255, 0 };
const uint8_t        ucGatewayAddressFLASH[4]         = { 10, 0, 0, 1 };    
const uint8_t        ucDNSServerAddressFLASH[4]       = { 10, 0, 0, 1 };  
const uint8_t        ucMACAddressFLASH[6]             = {0x34,0x4F,0x5C,0x00,0x00,0x00};

// Private function prototypes ************************************************

// Uses FreeRTOS+TCP to listen for incoming echo connections, creating a task
static void       prvEMACDeferredInterruptHandlerTask    ( void *pvParameters );
static void       ethTask                                ( void *pvParameters );

// ----------------------------------------------------------------------------
/// \brief     Initialise the TCP/IP stack
///
/// \param     none
///
/// \return    none
void init_tcp_ip_stack( void )
{   
   // initialise the TCP/IP stack.
   FreeRTOS_IPInit( ucIPAddressFLASH, ucNetMaskFLASH, ucGatewayAddressFLASH, ucDNSServerAddressFLASH, ucMACAddressFLASH );  
   
   // initialise receive complete task
   xTaskCreate( prvEMACDeferredInterruptHandlerTask, "bus receive", 2048, NULL, configMAX_PRIORITIES-2, &rxTaskToNotify );
}

// ----------------------------------------------------------------------------
/// \brief     delete all running ip tasks
///
/// \param     none
///
/// \return    none
void deinit_tcp_ip_stack( void )
{
   // delete all running ip tasks
   vTaskDelete(&rxTaskToNotify);
   vTaskDelete(&serverTask);
   xDeleteIPTask();
}

//-----------------------------------------------------------------------------
/// \brief     Called by FreeRTOS+TCP when the network connects or disconnects.  
///            Disconnect events are only received if implemented in the MAC 
///            driver.
///
/// \param     [in]  eIPCallbackEvent_t eNetworkEvent
///
/// \return    none
void vApplicationIPNetworkEventHook( eIPCallbackEvent_t eNetworkEvent )
{
uint32_t ulIPAddress, ulNetMask, ulGatewayAddress, ulDNSServerAddress;
char cBuffer[ 16 ];
static BaseType_t xTasksAlreadyCreated = pdFALSE;

	/* If the network has just come up...*/
	if( eNetworkEvent == eNetworkUp )
	{
		/* Create the tasks that use the IP stack if they have not already been
		created. */
		if( xTasksAlreadyCreated == pdFALSE )
		{
         // start webserver
         xTaskCreate( http_startHost, "http server listener", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, &serverTask );

         // set the task created flag
			xTasksAlreadyCreated = pdTRUE;
		}
	}
}

//-----------------------------------------------------------------------------
/// \brief     Utility function to generate a pseudo random number.
///
/// \param     -
///
/// \return    UBaseType_t random number
UBaseType_t uxRand( void )
{
	return bus_uart_tcp_getRandomNumber();
}

//-----------------------------------------------------------------------------
/// \brief     Seed random numbers to functions
///
/// \param     [in]  uint32_t ulSourceAddress
///            [in]  uint16_t usSourcePort
///            [in]  uint32_t ulDestinationAddress
///            [in]  uint16_t usDestinationPort
///
/// \return    none
extern uint32_t ulApplicationGetNextSequenceNumber( uint32_t ulSourceAddress, uint16_t usSourcePort, uint32_t ulDestinationAddress, uint16_t usDestinationPort )
{
	( void ) ulSourceAddress;
	( void ) usSourcePort;
	( void ) ulDestinationAddress;
	( void ) usDestinationPort;

	return uxRand();
}

#if( ipconfigUSE_LLMNR != 0 ) || ( ipconfigUSE_NBNS != 0 ) || ( ipconfigDHCP_REGISTER_HOSTNAME == 1 )
const char *pcApplicationHostnameHook( void )
{
   /* Assign the name "SB" to this network node.  This function will
   be called during the DHCP: the machine will be registered with an IP
   address plus this name. */
   return mainDEVICE_NICK_NAME;
}

const char *pcApplicationHostnameHookCAP( void )
{
   /* Assign the name "SB" to this network node.  This function will
   be called during the DHCP: the machine will be registered with an IP
   address plus this name. */
   return mainDEVICE_NICK_NAMEcapLetters;
}
#endif

#if( ipconfigUSE_LLMNR != 0 ) || ( ipconfigUSE_NBNS != 0 )
BaseType_t xApplicationDNSQueryHook( const char *pcName )
{
   static uint32_t llmnrsuccessCounter;
	BaseType_t     xReturn;

   // Determine if a name lookup is for this node.  Two names are given
   // to this node: that returned by pcApplicationHostnameHook() and that set
   // by mainDEVICE_NICK_NAME. If its a ip address query, check the ip address.
   if( strcmp( pcName, pcApplicationHostnameHook() ) == 0 || strcmp( pcName, mainDEVICE_NICK_NAMEcapLetters ) == 0 )
   {
      xReturn = pdPASS;
   }
   else if( strcmp( pcName, mainDEVICE_NICK_NAME ) == 0 || strcmp( pcName, mainDEVICE_NICK_NAMEcapLetters ) == 0 )
   {
      xReturn = pdPASS;
   }
   else
   {
      static char    ipAddrQuery[30];
      uint32_t       ipAddress;
      uint8_t        ip1DigitCnt = 1;
      uint8_t        ip1;
      uint8_t        ip2DigitCnt = 1;
      uint8_t        ip2;
      uint8_t        ip3DigitCnt = 1;
      uint8_t        ip3;
      uint8_t*       ipAddress8b;
      
      // get ip configuration
      FreeRTOS_GetAddressConfiguration( &ipAddress, NULL, NULL, NULL );
      ipAddress8b = (uint8_t*)(&ipAddress);
      
      // prepare string to be compared with query
      // get digit counts
      ip1 = ipAddress8b[0];
      while( ip1 >= 10 && ip1DigitCnt<4 )
      {
         ip1 /= 10;
         ip1DigitCnt++;
      }

      ip2 = ipAddress8b[1];
      while( ip2 >= 10 && ip2DigitCnt<4 )
      {
         ip2 /= 10;
         ip2DigitCnt++;
      }

      ip3 = ipAddress8b[2];
      while( ip3 >= 10 && ip3DigitCnt<4 )
      {
         ip3 /= 10;
         ip3DigitCnt++;
      }
         
      memset(ipAddrQuery, 0x00, 30);
      sprintf( ipAddrQuery, "%d%c%d%c%d%c%d\ain-addr%carpa", ipAddress8b[3], ip3DigitCnt, ipAddress8b[2], ip2DigitCnt, ipAddress8b[1], ip1DigitCnt, ipAddress8b[0], 0x04 );
   
      if( strcmp( pcName, ipAddrQuery ) == 0 )
      {
         llmnrsuccessCounter++;
         xReturn = pdPASS;
      }
      else
      {
         xReturn = pdFAIL;
      }
   }

   return xReturn;
}
#endif

// ----------------------------------------------------------------------------
/// \brief     called by the task.c freertos module
///
/// \param     none
///
/// \return    none
void vApplicationIdleHook( void )
{
   if( bus_uart_getFifoFlag() && processingIdle )
   {
      processingIdle = RESET;
      invoke_rx_task();
   }
   main_refreshWatchdog();
}

// ----------------------------------------------------------------------------
/// \brief     This task handles valid ethernet frames
///
/// \param     [in] void *pvParameters
///
/// \return    none
static void prvEMACDeferredInterruptHandlerTask( void *pvParameters )
{
   // current step get pointer and length of frame!
   NetworkBufferDescriptor_t *pxBufferDescriptor;
   size_t xBytesReceived;
   uint8_t* xFramePointer;
   // Used to indicate that xSendEventStructToIPTask() is being called because of an Ethernet receive event.
   IPStackEvent_t xRxEvent;
   BUS_UART_INPUT_ETHFRAME_t* frame;
   static uint32_t rxMacCallCounter, rxMacErrCounter;

   for( ;; )
   {
      /* Wait for the Ethernet MAC interrupt to indicate that another packet
      has been received.  The task notification is used in a similar way to a
      counting semaphore to count Rx events, but is a lot more efficient than
      a semaphore. */
      ulTaskNotifyTake( pdFALSE, portMAX_DELAY );
        
      rxMacCallCounter++;
        
      // search for the frame in the fifo buffer
      frame = bus_uart_getFrameFromFifo();
        
      if( frame == NULL )
      {
         processingIdle = SET;
         continue;
      }
        
      // set the bytes received variable
      xBytesReceived = frame->frameSize;
      xFramePointer = frame->frameStart;
      
      /* Allocate a network buffer descriptor that points to a buffer
      large enough to hold the received frame.  As this is the simple
      rather than efficient example the received data will just be copied
      into this buffer. */
      pxBufferDescriptor = (NetworkBufferDescriptor_t*)pxGetNetworkBufferWithDescriptor( xBytesReceived, 0 );
      
      if( pxBufferDescriptor != NULL )
      {
         // copy data onto the heap release frame from fifo
         memcpy(pxBufferDescriptor->pucEthernetBuffer, (uint8_t*)xFramePointer, xBytesReceived);
         bus_uart_ackFrameFromFifo();
         
         // set the pointer back
         pxBufferDescriptor->xDataLength = xBytesReceived;
         
         /* See if the data contained in the received Ethernet frame needs
         to be processed.  NOTE! It is preferable to do this in
         the interrupt service routine itself, which would remove the need
         to unblock this task for packets that don't need processing. */
         if( eConsiderFrameForProcessing( pxBufferDescriptor->pucEthernetBuffer ) == eProcessBuffer )
         {
            /* The event about to be sent to the TCP/IP is an Rx event. */
            xRxEvent.eEventType = eNetworkRxEvent;
            
            /* pvData is used to point to the network buffer descriptor that
            now references the received data. */
            xRxEvent.pvData = ( void * ) pxBufferDescriptor;
            
            /* Send the data to the TCP/IP stack. */
            if( xSendEventStructToIPTask( &xRxEvent, 0 ) == pdFALSE )
            {
               /* The buffer could not be sent to the IP task so the buffer
               must be released. */
               vReleaseNetworkBufferAndDescriptor( pxBufferDescriptor );
               
               /* Make a call to the standard trace macro to log the
               occurrence. */
               iptraceETHERNET_RX_EVENT_LOST();
            }
            else
            {
               /* The message was successfully sent to the TCP/IP stack.
               Call the standard trace macro to log the occurrence. */
               iptraceNETWORK_INTERFACE_RECEIVE();
            }
         }
         else
         {
            /* The Ethernet frame can be dropped, but the Ethernet buffer
            must be released. */
            vReleaseNetworkBufferAndDescriptor( pxBufferDescriptor );
         }
      }
      else
      {
         /* The event was lost because a network buffer was not available.
         Call the standard trace macro to log the occurrence. */
         iptraceETHERNET_RX_EVENT_LOST();
         rxMacErrCounter++;
      }

      // reset the processing frame flag
      processingIdle = SET;
   }
}

void invoke_rx_task( void )
{
   if( rxTaskToNotify != NULL )
   {
      // Notify to start the emac task to process the next frame from fifo
      xTaskNotifyGive( rxTaskToNotify );
   }
}

//-----------------------------------------------------------------------------
/// \brief     Function to send a tcp message
///
/// \param     [in]  char *pcBufferToTransmit
///                  const size_t xTotalLengthToSend
///
/// \return    none
void send_TCP( char *pcBufferToTransmit, const size_t xTotalLengthToSend )
{
struct freertos_sockaddr xRemoteAddress;
static uint32_t msgCounter = 0;
BaseType_t xAlreadyTransmitted = 0, xBytesSent = 0;
size_t xLenToSend;
Socket_t xSocket;

   /* Set the IP address (192.168.0.200) and sblink (1500) of the remote socket
   to which this client socket will transmit. */
   xRemoteAddress.sin_port = FreeRTOS_htons( 1500 );                 
   xRemoteAddress.sin_addr = FreeRTOS_inet_addr_quick( 192, 168, 0, 200 );
    
   /* create a socket */
   xSocket = create_TCP_client_socket();

   /* Connect to the remote socket.  The socket has not previously been bound to
   a local sblink number so will get automatically bound to a local sblink inside
   the FreeRTOS_connect() function. */
   if( FreeRTOS_connect( xSocket, &xRemoteAddress, sizeof( xRemoteAddress ) ) == 0 )
   {
       /* Keep sending until the entire buffer has been sent. */
       while( xAlreadyTransmitted < xTotalLengthToSend )
       {
           /* How many bytes are left to send? */
           xLenToSend = xTotalLengthToSend - xAlreadyTransmitted;
           xBytesSent = FreeRTOS_send( xSocket, &( pcBufferToTransmit[ xAlreadyTransmitted ] ), xLenToSend,  0 );
           if( xBytesSent >= 0 )
           {
               /* Data was sent successfully. */
               xAlreadyTransmitted += xBytesSent;
               vLoggingPrintf( "---Message successfully sended: %u---\n", msgCounter++ );
           }
           else
           {
               /* Error - break out of the loop for graceful socket close. */
               break;
           }
       }
   }

   /* Initiate graceful shutdown. */
   FreeRTOS_shutdown( xSocket, FREERTOS_SHUT_RDWR );

   /* Wait for the socket to disconnect gracefully (indicated by FreeRTOS_recv()
   returning a FREERTOS_EINVAL error) before closing the socket. */
   while( FreeRTOS_recv( xSocket, pcBufferToTransmit, xTotalLengthToSend, 0 ) >= 0 )
   {
       /* Wait for shutdown to complete.  If a receive block time is used then
       this delay will not be necessary as FreeRTOS_recv() will place the RTOS task
       into the Blocked state anyway. */
       vTaskDelay( 5 );

       /* Note - real applications should implement a timeout here, not just
       loop forever. */
   }

   /* The socket has shut down and is safe to close. */
   FreeRTOS_closesocket( xSocket );
}

//-----------------------------------------------------------------------------
/// \brief     Called if a call to pvPortMalloc() fails because there is insufficient
///         	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
///         	internally by FreeRTOS API functions that create tasks, queues, software
///         	timers, and semaphores.  The size of the FreeRTOS heap is set by the
///         	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. 
///
/// \param     none
///
/// \return    none
void vApplicationMallocFailedHook( void )
{
   static uint32_t malloc_fail_counter = 0;
   malloc_fail_counter++;
}

//-----------------------------------------------------------------------------
/// \brief     If ipconfigSUPSBLINK_OUTGOING_PINGS is set to 1 in 
///            FreeRTOSIPConfig.h then vApplicationPingReplyHook() is called by 
///            the IP stack when the stack receives a ping reply.
///
/// \param     [in]  ePingReplyStatus_t eStatus
/// \param     [in]  int16_t usIdentifier
///
/// \return    none
void vApplicationPingReplyHook( ePingReplyStatus_t eStatus, uint16_t usIdentifier )
{
    switch( eStatus )
    {
        case eSuccess    :
            /* A valid ping reply has been received.  Post the sequence number
            on the queue that is read by the vSendPing() function below.  Do
            not wait more than 10ms trying to send the message if it cannot be
            sent immediately because this function is called from the IP stack
            task - blocking in this function will block the IP stack. */
            //xQueueSend( xPingReplyQueue, &usIdentifier, 10 / sblinkTICK_PERIOD_MS );
            break;

        case eInvalidChecksum :
        case eInvalidData :
            /* A reply was received but it was not valid. */
            break;
    }
}

//-----------------------------------------------------------------------------
/// \brief     Init randon number generator
///
/// \param     none
///
/// \return    none
void bus_uart_tcp_rngInit( void )
{
	/* Enable RNG clock source */
	RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
	
	/* RNG Peripheral enable */
	RNG->CR |= RNG_CR_RNGEN;
}

//-----------------------------------------------------------------------------
/// \brief     Denit randon number generator
///
/// \param     none
///
/// \return    none
void bus_uart_tcp_rngDeInit( void )
{
	/* Enable RNG clock source */
	RCC->AHB2ENR &= ~RCC_AHB2ENR_RNGEN;
	
	/* RNG Peripheral enable */
	RNG->CR &= ~RNG_CR_RNGEN;
}

//-----------------------------------------------------------------------------
/// \brief     Return random number
///
/// \param     none
///
/// \return    RNG->DR
uint32_t bus_uart_tcp_getRandomNumber( void )
{
	/* Wait until one RNG number is ready */
	while (!(RNG->SR & (RNG_SR_DRDY)));

	/* Get a 32-bit Random number */
	return RNG->DR;
}

//-----------------------------------------------------------------------------
/// \brief     Returns pointer to mac address
///
/// \param     none
///
/// \return    ucMACAddressFLASH
const uint8_t* getMacAddress( void )
{
   return ucMACAddressFLASH;
}