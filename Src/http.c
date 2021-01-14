// ****************************************************************************
/// \file      http.c
///
/// \brief     HTTP Module
///
/// \details   This module contents HTTP functions to setup a server
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
#include  <string.h>
#include  <stdlib.h>
#include "http.h"
#include "printf.h"
#include "bus_uart_tcp.h"
#include "FreeRTOS_IP_Private.h"
#include "NetworkInterface.h"
#include "NetworkBufferManagement.h"
#include "FreeRTOS_DHCP.h"

// Private define *************************************************************
#define TXBUFFERSIZE       ( 15000u )
#define SMALLBUFFER        ( 250u )
#define URIBUFFER          ( 500u )

// Private types     **********************************************************

// Private variables **********************************************************
static uint32_t   guestCounter;

// the webpage top header
static const char *webpage_top = {
   "HTTP/1.1 200 OK\r\n"
   "Content-Type: text/html\r\n" //"Content-Type: text/html; charset=utf-8\r\n"
   "Keep-Alive: timeout=20\r\n" 
   "Connection: keep-alive\r\n\r\n"
   "<html><head>"
   "<title>Example</title>"
   "<style> div.main {"
   "font-family: Arial;"
   "padding: 0.01em 30px;"
   "box-shadow: 2px 2px 1px 1px #d2d2d2;"
   "background-color: #f1f1f1;}"
   "</style>"
   "</head>"
   "<body><div class='main'>"
   // Top header bar ////////////////////////////////////////////////////////
   //"<p><div style=\"height:72;border:1px solid #000;background-color: #000000;\">"
   "<p><div style=\"height:52;border:1px solid #000;background-color: #000000;\">"
   //RMLOGO
   "<font size=\"20\" font color=\"black\"><b>------</b></font>"
   "<font size=\"20\" font color=\"white\"><b>Example</b></font>"
   "</div></p>"
   //////////////////////////////////////////////////////////////////////////
   "<br />"
};

static const char *webpage_bottom_no_btn = {
   "<br>"    
   "</div></body></html>"
};

// Global variables ***********************************************************

// Private function prototypes ************************************************
static void        httpServerConnectionInstance    ( void *pvParameters );
static void        http_homepage                   ( uint8_t* pageBuffer, uint16_t pageBufferSize, Socket_t xConnectedSocket );
static void        http_205                        ( uint8_t* pageBuffer, uint16_t pageBufferSize, Socket_t xConnectedSocket );
static void        http_204                        ( uint8_t* pageBuffer, uint16_t pageBufferSize, Socket_t xConnectedSocket );
static void        http_201                        ( uint8_t* pageBuffer, uint16_t pageBufferSize, Socket_t xConnectedSocket );
static void        http_200                        ( uint8_t* pageBuffer, uint16_t pageBufferSize, Socket_t xConnectedSocket );
static void        http_301                        ( uint8_t* pageBuffer, uint16_t pageBufferSize, Socket_t xConnectedSocket );
static void        http_400                        ( uint8_t* pageBuffer, uint16_t pageBufferSize, Socket_t xConnectedSocket );
static uint16_t    http_favicon                    ( uint8_t* pageBuffer, uint16_t pageBufferSize, Socket_t xConnectedSocket );

// Private functions *********************************************************
// ----------------------------------------------------------------------------
/// \brief     Creates a task which listen on http port 80 for requests
///
/// \return    none
void http_startHost( void *pvParameters )
{
   struct freertos_sockaddr xClient, xBindAddress;
   Socket_t xListeningSocket, xConnectedSocket;
   socklen_t xSize = sizeof( xClient );
   const TickType_t xReceiveTimeOut = portMAX_DELAY;
   const BaseType_t xBacklog = 10;
   WinProperties_t xWinProps;
   
   /* Attempt to open the socket. */
   xListeningSocket = FreeRTOS_socket( FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP );

   /* Check the socket was created. */
   configASSERT( xListeningSocket != FREERTOS_INVALID_SOCKET );

   /* Set a time out so accept() will just wait for a connection. */
   FreeRTOS_setsockopt( xListeningSocket, 0, FREERTOS_SO_RCVTIMEO, &xReceiveTimeOut, sizeof( xReceiveTimeOut ) );

   /* Set the listening sblink to 80 as general for http. */
   xBindAddress.sin_port = ( uint16_t ) 80;
   xBindAddress.sin_port = FreeRTOS_htons( xBindAddress.sin_port );

   /* Bind the socket to the sblink that the client RTOS task will send to. */
   FreeRTOS_bind( xListeningSocket, &xBindAddress, sizeof( xBindAddress ) );

   /* Set the socket into a listening state so it can accept connections.
   The maximum number of simultaneous connections is limited to 20. */
   FreeRTOS_listen( xListeningSocket, xBacklog );

   for( ;; )
   {
      /* Wait for incoming connections. */
      xConnectedSocket = FreeRTOS_accept( xListeningSocket, &xClient, &xSize );
      configASSERT( xConnectedSocket != FREERTOS_INVALID_SOCKET );

      /* Spawn a RTOS task to handle the connection. */
      xTaskCreate( httpServerConnectionInstance, "http server", 1024, ( void * ) xConnectedSocket, tskIDLE_PRIORITY+1, NULL );
   }
}

static void httpServerConnectionInstance( void *pvParameters )
{
   const TickType_t  xReceiveTimeOut = pdMS_TO_TICKS( 4000 );
   const TickType_t  xSendTimeOut = pdMS_TO_TICKS( 4000 );
   const BaseType_t  reuseSocket = pdTRUE;
   const uint16_t    max_uri_len = URIBUFFER;
   static uint8_t    serverInstanceMallocError;
   Socket_t          xConnectedSocket;
   uint8_t           uri[URIBUFFER];
   uint8_t           *sp1, *sp2; 
   TickType_t        xTimeOnShutdown;
   uint8_t           *pucRxBuffer;
   uint8_t           *pageBuffer;
   uint32_t          counterDescriptorAllocationFail;
   static volatile BaseType_t lengthOfbytes;
   static uint16_t   etimeout;  
   static uint16_t   enomem;  
   static uint16_t   enotconn;
   static uint16_t   eintr;   
   static uint16_t   einval; 
   static uint16_t   uriTooLongError;
   WinProperties_t xWinProps;
   
   // get the socket
   xConnectedSocket = ( Socket_t ) pvParameters;

	/* Attempt to create the buffer used to receive the string to be echoed
	back.  This could be avoided using a zero copy interface that just returned
	the same buffer. */
	pucRxBuffer = ( uint8_t * ) pvPortMalloc( ipconfigTCP_MSS );
   pageBuffer = ( uint8_t * ) pvPortMalloc( TXBUFFERSIZE );

	if( pucRxBuffer != 0 && pageBuffer != 0 )
	{
		FreeRTOS_setsockopt( xConnectedSocket, 0, FREERTOS_SO_RCVTIMEO, &xReceiveTimeOut, sizeof( xReceiveTimeOut ) );
		FreeRTOS_setsockopt( xConnectedSocket, 0, FREERTOS_SO_SNDTIMEO, &xSendTimeOut, sizeof( xReceiveTimeOut ) );
   }
   else
   {
      serverInstanceMallocError++;
      vTaskDelete( NULL );
   }

   for( ;; )
   {
      // Receive data on the socket.
      lengthOfbytes = FreeRTOS_recv( xConnectedSocket, pucRxBuffer, ipconfigTCP_MSS, 0 );
         
      // check lengthOfbytes ------------- lengthOfbytes > 0                           --> data received
      //                                   lengthOfbytes = 0                           --> timeout
      //                                   lengthOfbytes = pdFREERTOS_ERRNO_ENOMEM     --> not enough memory on socket
      //                                   lengthOfbytes = pdFREERTOS_ERRNO_ENOTCONN   --> socket was or got closed
      //                                   lengthOfbytes = pdFREERTOS_ERRNO_EINTR      --> if the socket received a signal, causing the read operation to be aborted
      //                                   lengthOfbytes = pdFREERTOS_ERRNO_EINVAL     --> socket is not valid
      if( lengthOfbytes > 0 )
      {         
         // check for a GET request
         if (!strncmp((char const*)pucRxBuffer, "GET ", 4)) {
            // extract URI
            sp1 = pucRxBuffer + 4;
            sp2 = memchr(sp1, ' ', max_uri_len);
            uint16_t len = sp2 - sp1;
            if(len>URIBUFFER)
            {
               uriTooLongError++;
               FreeRTOS_shutdown( xConnectedSocket, FREERTOS_SHUT_RDWR );                                                           
               break;
            }
            memset(uri, 0x00, URIBUFFER);
            memcpy(uri, sp1, len);
            uri[len] = '\0';
            
            if(strncmp((char const*)uri, "/home", 5u) == 0)
            {                     
               // build and send webpage
               http_homepage(pageBuffer, TXBUFFERSIZE, xConnectedSocket);
            }
            else if(strncmp((char const*)uri, "/favicon.ico", 13u) == 0)
            {
               // uri not known, send 301 - redirect
               http_favicon(pageBuffer, TXBUFFERSIZE, xConnectedSocket);
            }
            else
            {
               // uri not known, send 301 - redirect
               http_301(pageBuffer, SMALLBUFFER, xConnectedSocket);
            }
         }
         
         // check for a POST request
         if (!strncmp((char const*)pucRxBuffer, "POST ", 5u)) {
            
            // extract URI
            sp1 = pucRxBuffer + 5;
            sp2 = memchr(sp1, ' ', max_uri_len);
            uint16_t len = sp2 - sp1;
            if(len>URIBUFFER)
            {
               uriTooLongError++;
               FreeRTOS_shutdown( xConnectedSocket, FREERTOS_SHUT_RDWR );                                                           
               break;
            }
            memset(uri, 0x00, URIBUFFER);
            memcpy(uri, sp1, len);
            uri[len] = '\0';
            
            if (strncmp((char const*)uri, "/led_on", 7u) == 0)
            {
               // led on
               // ...
               
               // no content response
               http_204(pageBuffer, SMALLBUFFER, xConnectedSocket);
            }
            else
            {
               // send bad request if the uri is unkown or faulty
               http_400(pageBuffer, SMALLBUFFER, xConnectedSocket);
            }
         }
   
         // Closing handhsake
         FreeRTOS_shutdown( xConnectedSocket, FREERTOS_SHUT_RDWR );
         //break;
      }
      else if( lengthOfbytes == 0 )
      {
         // No data was received, but FreeRTOS_recv() did not return an error. Timeout?
      }
      else if( lengthOfbytes == pdFREERTOS_ERRNO_ENOMEM )                                                                                        
      {                                                                                                                    
         // Error (maybe the connected socket already shut down the socket?). Attempt graceful shutdown.                   
         enomem++;
         FreeRTOS_shutdown( xConnectedSocket, FREERTOS_SHUT_RDWR );                                                        
         break;
      } 
      else if( lengthOfbytes == pdFREERTOS_ERRNO_ENOTCONN )                                                                   
      {                                                                                                                       
         // Error (maybe the connected socket already shut down the socket?). Attempt graceful shutdown.                      
         enotconn++;
         FreeRTOS_shutdown( xConnectedSocket, FREERTOS_SHUT_RDWR );                                                           
         break;
      } 
      else if( lengthOfbytes == pdFREERTOS_ERRNO_EINTR )                                                                      
      {                                                                                                                       
         // Error (maybe the connected socket already shut down the socket?). Attempt graceful shutdown.                      
         eintr++;
         FreeRTOS_shutdown( xConnectedSocket, FREERTOS_SHUT_RDWR );                                                           
         break;
      } 
      else if( lengthOfbytes == pdFREERTOS_ERRNO_EINVAL )                                                                      
      {                                                                                                                       
         // Error (maybe the connected socket already shut down the socket?). Attempt graceful shutdown.                      
         einval++;
         FreeRTOS_shutdown( xConnectedSocket, FREERTOS_SHUT_RDWR );                                                           
         break;
      } 
      else
      {                                                                                                                       
         // Error (maybe the connected socket already shut down the socket?). Attempt graceful shutdown.                      
         FreeRTOS_shutdown( xConnectedSocket, FREERTOS_SHUT_RDWR );                                                           
         break;
      } 
   }
   
   // The RTOS task will get here if an error is received on a read.  Ensure the
   // socket has shut down (indicated by FreeRTOS_recv() returning a FREERTOS_EINVAL
   // error before closing the socket).
   // Wait for the shutdown to take effect, indicated by FreeRTOS_recv()
   // returning an error.
   xTimeOnShutdown = xTaskGetTickCount();
   do
   {
      if( FreeRTOS_recv( xConnectedSocket, pucRxBuffer, ipconfigTCP_MSS, 0 ) < 0 )
      {
         vTaskDelay( pdMS_TO_TICKS( 250 ) );
         break;
      }
   } while( ( xTaskGetTickCount() - xTimeOnShutdown ) < pdMS_TO_TICKS( 5000 ) );
   
   /* Finished with the socket, buffer, the task. */
   vPortFree( pucRxBuffer );
   vPortFree( pageBuffer );
   FreeRTOS_closesocket( xConnectedSocket );

   // delete the task
   vTaskDelete( NULL );
}

static void http_homepage( uint8_t* pageBuffer, uint16_t pageBufferSize, Socket_t xConnectedSocket )
{
   uint16_t 		stringLength;
   float    		temperature;
   float    		voltage;
   uint32_t 		totalSeconds;
   uint32_t 		days;
   uint32_t 		hours;
   uint32_t 		minutes;
   uint32_t 		seconds;
   uint8_t  		*ipAddress8b;
   uint32_t 		ipAddress;
   uint32_t 		netMask;
   uint32_t 		dnsAddress;
   uint32_t 		gatewayAddress;
   const uint8_t* 	stackMacAddress;
   
   static const char *webpage_home = {
      "<p><h3>Example</h3></p>"
      "<p>IP: %d.%d.%d.%d</p>"
      "<p>MAC: %02x:%02x:%02x:%02x:%02x:%02x</p>"
      "<p>Uptime: %d days, %d hours, %d minutes, %d seconds</p>"
      "<p>Free heap: %d bytes</p>"
      "<p><form action=\"led_on\" method=\"post\"><button  style=\"width:200px\">Led on</button></form></p>"
      "<p></p>"
      "<p>Guest counter: %d</p>" 
      "<br />"
   };
   
   // set the placeholder in every webpage fragment and put all fragments together
   // top header
   stringLength = snprintf(0, 0, webpage_top);
   snprintf((char*)pageBuffer, stringLength, webpage_top);
   
   // send fragment of the webpage//////////////////////////////////////////////
   if( stringLength < pageBufferSize )
   {
      FreeRTOS_send( xConnectedSocket, pageBuffer, stringLength, 0 );
   }
   /////////////////////////////////////////////////////////////////////////////
   
   // monitor
   FreeRTOS_GetAddressConfiguration( &ipAddress, &netMask, &gatewayAddress, &dnsAddress );
   ipAddress8b    = (uint8_t*)(&ipAddress);
   stackMacAddress = FreeRTOS_GetMACAddress();
   totalSeconds   = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
   days           = (totalSeconds / 86400);       
   hours          = (totalSeconds / 3600) % 24;   
   minutes        = (totalSeconds / 60) % 60;     
   seconds        = totalSeconds % 60;            
   guestCounter++;
   stringLength = snprintf(0, 0, webpage_home, ipAddress8b[0], ipAddress8b[1], ipAddress8b[2], ipAddress8b[3], stackMacAddress[0], stackMacAddress[1], stackMacAddress[2], stackMacAddress[3], stackMacAddress[4], stackMacAddress[5], days, hours, minutes, seconds, (int) xPortGetFreeHeapSize(), guestCounter );
   snprintf((char*)(pageBuffer), stringLength, webpage_home, ipAddress8b[0], ipAddress8b[1], ipAddress8b[2], ipAddress8b[3], stackMacAddress[0], stackMacAddress[1], stackMacAddress[2], stackMacAddress[3], stackMacAddress[4], stackMacAddress[5], days, hours, minutes, seconds, (int) xPortGetFreeHeapSize(), guestCounter );
   
   // send fragment of the webpage//////////////////////////////////////////////
   if( stringLength < pageBufferSize )
   {
      FreeRTOS_send( xConnectedSocket, pageBuffer, stringLength, 0 );
   }
   /////////////////////////////////////////////////////////////////////////////
   
   // bottom
   stringLength = snprintf(0, 0, webpage_bottom_no_btn);
   snprintf((char*)pageBuffer, stringLength, webpage_bottom_no_btn);
   
   // send fragment of the webpage//////////////////////////////////////////////
   if( stringLength < pageBufferSize )
   {
      FreeRTOS_send( xConnectedSocket, pageBuffer, stringLength, 0 );
   }
   /////////////////////////////////////////////////////////////////////////////
}

// status code: ok
static void http_200( uint8_t* pageBuffer, uint16_t pageBufferSize, Socket_t xConnectedSocket )
{
   // general page variables
   uint16_t stringLength;
   static const char *httpCode200 = {
      "HTTP/1.1 200 OK\r\n"
      "Content-Length: 0\r\n\r\n"
   };
   
   // generate message
   stringLength = snprintf(0, 0, httpCode200);
   snprintf((char*)pageBuffer, stringLength+1, httpCode200);
   
   // send fragment of the webpage
   if( stringLength < pageBufferSize )
   {
      FreeRTOS_send( xConnectedSocket, pageBuffer, stringLength, 0 );
   }
}

// status code: no content
static void http_204( uint8_t* pageBuffer, uint16_t pageBufferSize, Socket_t xConnectedSocket )
{
   // general page variables
   uint16_t stringLength;
   static const char *httpCode204 = {
      "HTTP/1.1 204 No Content\r\n"
      "Connection: close\r\n\r\n"
   };
   
   // generate message
   stringLength = snprintf(0, 0, httpCode204);
   snprintf((char*)pageBuffer, stringLength+1, httpCode204);
   
   // send fragment of the webpage
   if( stringLength < pageBufferSize )
   {
      FreeRTOS_send( xConnectedSocket, pageBuffer, stringLength, 0 );
   }
}

// status code: reset ui
static void http_205( uint8_t* pageBuffer, uint16_t pageBufferSize, Socket_t xConnectedSocket )
{
   // general page variables
   uint16_t stringLength;
   static const char *httpCode205 = {
      "HTTP/1.1 205 Reset Content\r\n"
      "Content-Length: 0\r\n\r\n"
   };
   
   // generate message
   stringLength = snprintf(0, 0, httpCode205);
   snprintf((char*)pageBuffer, stringLength+1, httpCode205);
   
   // send fragment of the webpage
   if( stringLength < pageBufferSize )
   {
      FreeRTOS_send( xConnectedSocket, pageBuffer, stringLength, 0 );
   }
}

// status code: created
static void http_201( uint8_t* pageBuffer, uint16_t pageBufferSize, Socket_t xConnectedSocket )
{
   // general page variables
   uint16_t stringLength;
   static const char *httpCode201 = {
      "HTTP/1.1 201 Created\r\n"
      //"Refresh: 0\r\n"
      "Refresh: 0, url= \r\n"
      "Content-Length: 0\r\n\r\n"
   };
   
   // generate message
   stringLength = snprintf(0, 0, httpCode201);
   snprintf((char*)pageBuffer, stringLength+1, httpCode201);
   
   // send fragment of the webpage
   if( stringLength < pageBufferSize )
   {
      FreeRTOS_send( xConnectedSocket, pageBuffer, stringLength, 0 );
   }
}

// status code: bad request
static void http_400( uint8_t* pageBuffer, uint16_t pageBufferSize, Socket_t xConnectedSocket )
{
   // general page variables
   uint16_t stringLength;
   static const char *httpCode400 = {
      "HTTP/1.1 400 Bad Request\r\n"
      "Content-Length: 0\r\n\r\n"
   };
   
   // generate message
   stringLength = snprintf(0, 0, httpCode400);
   snprintf((char*)pageBuffer, stringLength+1, httpCode400);
   
   // send fragment of the webpage
   if( stringLength < pageBufferSize )
   {
      FreeRTOS_send( xConnectedSocket, pageBuffer, stringLength, 0 );
   }
}

// status code: moved permanently
static void http_301( uint8_t* pageBuffer, uint16_t pageBufferSize, Socket_t xConnectedSocket )
{
   // general page variables
   uint16_t stringLength;
   static const char *httpCode301 = {
      "HTTP/1.1 301 Moved Permanently\r\n"
      "Location: /home\r\n" 
      "Content-Length: 0\r\n\r\n"
   };

   // generate message
   stringLength = snprintf(0, 0, httpCode301 );
   snprintf((char*)pageBuffer, stringLength+1, httpCode301 );
   
   // send fragment of the webpage
   if( stringLength < pageBufferSize )
   {
      FreeRTOS_send( xConnectedSocket, pageBuffer, stringLength, 0 );
   }
}

static uint16_t http_favicon( uint8_t* pageBuffer, uint16_t pageBufferSize, Socket_t xConnectedSocket )
{
   // general page variables
   uint16_t stringLength;
   static const char *httpFavicon = {
   "HTTP/1.1 200 OK\r\n"
   "Content-Type: text/html\r\n" //"Content-Type: text/html; charset=utf-8\r\n"
   "Connection: close\r\n\r\n"
   "<html><head>"
   "<link rel=\"shortcut icon\" href=\"data:image/gif;base64,R0lGODlhIAAgAHAAACwAAAAAIAAgAIf////s2+Pqt8j/wNT/v836wsv/xM7/wM7/w9L6ws38uMP4ZHTQCyfgAB7cABnXBhzeBh7bABjiAR/iAB3UAC3wdYfZAwvdAybkAyHqBivjABrgAyblASbiASHjAiLkACXwZ4XkABHsBDTgBCjVASnfASbgBS3eACnhACjiASnXBR7tfpHdAxHpAC3sAifiACXtACXiACbfACXjASfjABnmY3XbBgrYAB7qAB3tAB3tABrgAB3iAB7hAB7ZCCX31+z7vbLjycztw8T0v8fxw8Puysr3vsf1wMj/s6/6v93+m57oo7Xxnaz/nrX3m6rworD9mbH5nLH6nbL1pKPhaXLbBQPfABvhABDrAx3pABDeABDjABbfABXgARbdASXgfY/iABD9AjfcACLTByjoByfgAiffAyfkABvya4ngABDvAzPUAyLfASjjAiTlACndASfeACXkARzleIvZBRHqACvhACLoACjeASDnASjdBCHpaH7gDhfkAyvkBSLqACfeBSDgBybnBSrkBCbjACn67//3x7vz2N354Nz/0+D62t310tj72dr62Nn9zcH4rc7zgobqjKTskJ/6gaDvjqLsiKDwip/viZ7lhYnuanjjDhDjACDhABfmABjdARrnAh/hABniABruAS/ddILTBAzqACjiBiLsCC3hASPkAyPlBCTYAh72b4/iABXwBDXUAyPeACvlACvdACvdACbeACfgAinhBR7vdXLVABnnABvSARflBSDdBB/mAB/vaHnmEz70FULfFzvsFD3lFD7pCzrnEDrVFDMAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAI/wABCBxIsKDBgwgTKlzIsKHDhxAjPvwhABEBAgYIWDTAsWPGhwEcXeR4caQAjCcVPKyRS4uWllrEuJQpE4aWNQ8rxBSjRowYGD6BAv0p5iEILT19xopFtKnPh5a0wOgJYyhPnz3VaFlZZU2WqkFpii3q8IeCIEWKFCoiwMDJIigJAHmYRMkTJk2gPIHCly9evlMeUqniMksWwi4TJybbUJSWMGLWXFUqhinTrQ6PqlkTayhTykEfOhaK9SfVqU8zZ2I6VM1Qq0AhFklEKFGiQwFu07Ztm9FDRxUsWZIECZJw4Y+OW7L1sBfiLC5tztQCHanoUVl9EgUrNJdRpKU/Ky5dyvShLTFZcml/7TPLT8wNexX7JezXfDXFiOknZp9YMYkABijggAQWaOCBAAQEADs=\" />"   
   "</body><body>"
   "</head></html>"
   };
   
   // generate message
   stringLength = snprintf(0, 0, httpFavicon);
   snprintf((char*)pageBuffer, stringLength, httpFavicon);
   
   // send fragment of the webpage
   if( stringLength < pageBufferSize )
   {
      FreeRTOS_send( xConnectedSocket, pageBuffer, stringLength, 0 );
   }
   return stringLength;
}