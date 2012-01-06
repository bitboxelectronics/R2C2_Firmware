#include "alerter.h"

#include "config.h"

#include "buzzer.h"


#define BEEP_TONE 440
#define BEEP_TIME 200

void alert_event(int32_t beep_event)
{
  if ((config.beep_on_events & beep_event) != 0)
  {
    buzzer_play (BEEP_TONE, BEEP_TIME);
  }
}

