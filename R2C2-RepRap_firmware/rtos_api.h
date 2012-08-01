/* Copyright (c) 2012 Bob Cousins bobcousins42@googlemail.com              */
/* **************************************************************************
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/
// **************************************************************************
// Description:
//
// **************************************************************************

#ifndef _RTOS_API_H
#define _RTOS_API_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "app/machine.h"

#ifdef USE_FREERTOS
  /* FreeRTOS includes. */
  #include "FreeRTOS.h"
  #include "task.h"
  #include "queue.h"
#else
  
  
// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

#define tskIDLE_PRIORITY 0

#define portBASE_TYPE	long

#define portMAX_DELAY 0xffffffff

#define pdTRUE		( 1 )
#define pdFALSE		( 0 )

#define pdPASS									( 1 )
#define pdFAIL									( 0 )
#define errQUEUE_EMPTY							( 0 )
#define errQUEUE_FULL							( 0 )

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

typedef void (*pdTASK_CODE)( void * );
typedef void * xTaskHandle;


typedef unsigned long xQueueHandle ;

typedef unsigned long portTickType;

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

portBASE_TYPE xTaskCreate(
							  pdTASK_CODE pvTaskCode,
							  const char * const pcName,
							  unsigned short usStackDepth,
							  void *pvParameters,
							  unsigned portBASE_TYPE uxPriority,
							  xTaskHandle *pvCreatedTask
						  );

void vTaskDelete( xTaskHandle pxTask );

void vTaskStartScheduler( void );


xQueueHandle xQueueCreate( unsigned portBASE_TYPE uxQueueLength, unsigned portBASE_TYPE uxItemSize );

unsigned portBASE_TYPE uxQueueMessagesWaiting( const xQueueHandle xQueue );

signed portBASE_TYPE xQueueReceive( xQueueHandle xQueue, void * const pvBuffer, portTickType xTicksToWait );

signed portBASE_TYPE xQueueSend( xQueueHandle xQueue, const void * const pvItemToQueue, portTickType xTicksToWait);


signed portBASE_TYPE xQueueGenericReceive( xQueueHandle xQueue, void * const pvBuffer, portTickType xTicksToWait, portBASE_TYPE xJustPeek );

signed portBASE_TYPE xQueueGenericSend( xQueueHandle xQueue, const void * const pvItemToQueue, portTickType xTicksToWait, portBASE_TYPE xCopyPosition );

#endif


#endif // _RTOS_API_H
