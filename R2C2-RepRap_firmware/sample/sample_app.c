/* 	A sample application for R2C2.

	Blink the X stepper LED.
*/

#include "r2c2.h"

tTimer blinkTimer;

int led_on;

void blinkTimerCallback (tTimer *pTimer)
{
    led_on = led_on ^ 1;

	digital_write (1, (1<<20), led_on);
}

int app_main (void)
{
  	buzzer_init();

 	buzzer_play(1500, 100); /* low beep */
	buzzer_wait();
  	buzzer_play(2500, 200); /* high beep */

  	pin_mode(1, (1 << 20), OUTPUT);

	AddSlowTimer (&blinkTimer);
	blinkTimer.AutoReload = 1;
	StartSlowTimer (&blinkTimer, 500, blinkTimerCallback);
	  
	while (1);   
  
}
