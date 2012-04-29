#include "queue.h"

#include "gcode_parse.h"

typedef struct {
    tLineBuffer *pLineBuf;
} tGcodeInputMsg;

extern xQueueHandle GcodeRxQueue;

void GcodeTask( void *pvParameters );



