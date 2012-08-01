
#include "rtos_api.h"



portBASE_TYPE xTaskCreate(
							  pdTASK_CODE pvTaskCode,
							  const char * const pcName,
							  unsigned short usStackDepth,
							  void *pvParameters,
							  unsigned portBASE_TYPE uxPriority,
							  xTaskHandle *pvCreatedTask
						  )
{
  return pdPASS;
}

void vTaskDelete( xTaskHandle pxTask )
{
}

void vTaskStartScheduler( void )
{
}



xQueueHandle xQueueCreate( unsigned portBASE_TYPE uxQueueLength, unsigned portBASE_TYPE uxItemSize )
{
  return 0;
}


unsigned portBASE_TYPE uxQueueMessagesWaiting( const xQueueHandle xQueue )
{
  return 0;
}

signed portBASE_TYPE xQueueReceive( xQueueHandle xQueue, void * const pvBuffer, portTickType xTicksToWait )
{
  return pdTRUE;
}


signed portBASE_TYPE xQueueSend( xQueueHandle xQueue, const void * const pvItemToQueue, portTickType xTicksToWait)
{
  return pdTRUE;
}



signed portBASE_TYPE xQueueGenericReceive( xQueueHandle xQueue, void * const pvBuffer, portTickType xTicksToWait, portBASE_TYPE xJustPeek )
{
  return pdTRUE;
}


signed portBASE_TYPE xQueueGenericSend( xQueueHandle xQueue, const void * const pvItemToQueue, portTickType xTicksToWait, portBASE_TYPE xCopyPosition )
{
  return pdTRUE;
}

