/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Modifications Copyright (c) 2011 Sungeun K. Jeon
  
  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#if 0
//#include <util/delay.h>
//#include <avr/interrupt.h>
#else
#include "lpc17xx_timer.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_pinsel.h"

#include "timer.h"
#include "pinout.h"
#include "ios.h"
#include "temp.h"
#endif

#include "stepper.h"
#include "config.h"
//#include "settings.h"
//#include "nuts_bolts.h"
#include "planner.h"
//#include "limits.h"

#include "endstops.h"

// Some useful constants
#define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
#define DIRECTION_MASK  ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
#define STEPPING_MASK   (STEP_MASK | DIRECTION_MASK) // All stepping-related bits (step/direction)

#define TICKS_PER_MICROSECOND (F_CPU/1000000)
#define CYCLES_PER_ACCELERATION_TICK ((TICKS_PER_MICROSECOND*1000000)/ACCELERATION_TICKS_PER_SECOND)

// Globals
      
volatile uint16_t steptimeout = 0;

static uint8_t  led_count [NUM_AXES];
static uint8_t  led_on;         // a bit mask
static uint8_t  leds_enabled;
static uint16_t led_on_time;
static uint16_t led_off_time;

static tTimer blinkTimer;


// Locals

// DAC scale is calculated as voltage range * 10 (3.3V=33/10) and seconds, steps per mm, and finally mm/s per volt divided by DAC range (10 bits)
const uint32_t mm_per_sec_per_volt = 200;
uint32_t dac_scale;


static block_t *current_block;  // A pointer to the block currently being traced

// Variables used by The Stepper Driver Interrupt
//static uint8_t out_bits;        // The next stepping-bits to be output
static uint32_t direction_bits; // all axes (different ports)    
static uint32_t step_bits_e;    // for extruder     
static uint32_t step_bits_xyz;  // for XYZ steppers (same port)

static int32_t counter_x,       // Counter variables for the bresenham line tracer
               counter_y, 
               counter_z;       
static int32_t counter_e;       
static uint32_t step_events_completed; // The number of step events executed in the current block
static volatile int busy; // true when SIG_OUTPUT_COMPARE1A is being serviced. Used to avoid retriggering that handler.

// Variables used by the trapezoid generation
static uint32_t cycles_per_step_event;        // The number of machine cycles between each step event
static uint32_t trapezoid_tick_cycle_counter; // The cycles since last trapezoid_tick. Used to generate ticks at a steady
                                              // pace without allocating a separate timer
static uint32_t trapezoid_adjusted_rate;      // The current rate of step_events according to the trapezoid generator
static uint32_t min_safe_rate;  // Minimum safe rate for full deceleration rate reduction step. Otherwise halves step_rate.
//static uint8_t cycle_start;     // Cycle start flag to indicate program start and block processing.

static volatile uint8_t    accel_flag;

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
// 
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates by block->rate_delta
//  during the first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is always +/- block->rate_delta and is applied at a constant rate following the midpoint rule
//  by the trapezoid generator, which is called ACCELERATION_TICKS_PER_SECOND times per second.

static void set_step_events_per_minute(uint32_t steps_per_minute);

//
// === Platform dependent: for R2C2
//
static inline void inc_led_count (uint8_t *pCount, uint8_t led_mask)
{
#ifdef STEP_LED_FLASH_VARIABLE
  (*pCount) ++;
  if (*pCount == 128)
  {
    led_on = led_on ^ led_mask;
    *pCount = 0;
  }
#endif
}

static inline void  set_direction_pins (void) 
{
  // x_direction( (direction_bits & (1<<X_DIRECTION_BIT))?0:1);

  if (direction_bits & X_DIR_PIN)
    GPIO_ClearValue (X_DIR_PORT, X_DIR_PIN);
  else
    GPIO_SetValue (X_DIR_PORT, X_DIR_PIN);
  
  // y_direction( (direction_bits & (1<<Y_DIRECTION_BIT))?0:1);
  if (direction_bits & Y_DIR_PIN)
    GPIO_ClearValue (Y_DIR_PORT, Y_DIR_PIN);
  else
    GPIO_SetValue (Y_DIR_PORT, Y_DIR_PIN);
      
  // z_direction( (direction_bits & (1<<Z_DIRECTION_BIT))?0:1);
  if (direction_bits & Z_DIR_PIN)
    GPIO_ClearValue (Z_DIR_PORT, Z_DIR_PIN);
  else
    GPIO_SetValue (Z_DIR_PORT, Z_DIR_PIN);

  // e_direction( (direction_bits & (1<<E_DIRECTION_BIT))?0:1);
  if (direction_bits & E_DIR_PIN)
    GPIO_ClearValue (E_DIR_PORT, E_DIR_PIN);
  else
    GPIO_SetValue (E_DIR_PORT, E_DIR_PIN);
    
    
}

// step selected pins (output high)
static inline void  set_step_pins (void) 
{
  // XYZ Steppers on same port
#ifdef STEP_LED_FLASH_VARIABLE
  if (step_bits_xyz & (1<<X_STEP_BIT))
  {
    inc_led_count (&led_count[X_AXIS], (1<<X_AXIS));
  }
  if (step_bits_xyz & (1<<Y_STEP_BIT))
  {
    inc_led_count (&led_count[Y_AXIS], (1<<Y_AXIS));
  }  
  if (step_bits_xyz & (1<<Z_STEP_BIT))
  {
    inc_led_count (&led_count[Z_AXIS], (1<<Z_AXIS));
  }
#endif
  
  GPIO_SetValue (X_STEP_PORT, step_bits_xyz);
    
  // extruder stepper    
  if (step_bits_e)
  {
#ifdef STEP_LED_FLASH_VARIABLE
    if (step_bits_e & (1<<E_STEP_BIT))
    {
      inc_led_count (&led_count[E_AXIS], (1<<E_AXIS));
    }
#endif  
  
    GPIO_SetValue (E_STEP_PORT, step_bits_e);
  }
  
}

// unstep all stepper pins (output low)
static inline void  clear_all_step_pins (void) 
{
  // Note: XYZ on same port 
  GPIO_ClearValue (X_STEP_PORT, X_STEP_PIN | Y_STEP_PIN | Z_STEP_PIN);
  
  GPIO_ClearValue (E_STEP_PORT, E_STEP_PIN);
}

// unstep selected pins
static inline void  clear_step_pins (void) 
{
  // XYZ Steppers on same port
  GPIO_ClearValue (X_STEP_PORT, step_bits_xyz);
    
  // extruder stepper    
  if (step_bits_e)
  {
    GPIO_ClearValue (E_STEP_PORT, step_bits_e);
  }
}

// unstep pins according to led_on bit mask
static inline void  clear_step_pins_by_state (void) 
{
  uint32_t step_pins = 0;
  
  // can turn off stepper pins but must NOT turn on 
  // stepper pins because it would cause an unwanted step
  
  if ((led_on & (1<<X_AXIS)) == 0)
  {
    step_pins |= X_STEP_PIN;
  }
  if ((led_on & (1<<Y_AXIS)) == 0)
  {
    step_pins |= Y_STEP_PIN;
  }
  if ((led_on & (1<<Z_AXIS)) == 0)
  {
    step_pins |= Z_STEP_PIN;
  }
  
  // XYZ Steppers on same port
  GPIO_ClearValue (X_STEP_PORT, step_pins);

  if ((led_on & (1<<E_AXIS)) == 0)
  {
    GPIO_ClearValue (E_STEP_PORT, E_STEP_PIN);
  }
}

void startBlink(void)
{
  leds_enabled = 1;
#ifdef STEP_LED_FLASH_FIXED  
  StartSlowTimer (&blinkTimer, led_on_time, blinkTimerCallback);
  led_on = 0x0F;
#else
  led_on = 0x00;
#endif
}

void stopBlink (void)
{
  leds_enabled = 0;
  led_on = 0x00;
  StopSlowTimer (&blinkTimer);
  unstep();
}

void blinkTimerCallback (tTimer *pTimer)
{
  if (leds_enabled)
  {
    led_on = led_on ^ 0x0F;

    if (led_on)
      StartSlowTimer (&blinkTimer, led_on_time, blinkTimerCallback);
    else
      StartSlowTimer (&blinkTimer, led_off_time, blinkTimerCallback);
  }
  else
  {
    led_on = 0x00;
  }
}

//
// =========================
//

void st_wake_up() {
#if 0
  // Enable steppers by resetting the stepper disable port
  STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT);
  // Enable stepper driver interrupt
  TIMSK1 |= (1<<OCIE1A);
#else

  enableHwTimer(1);
  
  dac_scale = (33 * 60 * config.steps_per_mm_x * mm_per_sec_per_volt) / 1024/10;

#endif
}

static void st_go_idle() {
#if 0
  // Force stepper dwell to lock axes for a defined amount of time to ensure the axes come to a complete
  // stop and not drift from residual inertial forces at the end of the last movement.
  _delay_ms(STEPPER_IDLE_LOCK_TIME);   
  // Disable steppers by setting stepper disable
  STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT);
  // Disable stepper driver interrupt
  TIMSK1 &= ~(1<<OCIE1A); 
#else
  disableHwTimer(1);
  clear_all_step_pins();
#endif
}

// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
static void trapezoid_generator_reset() 
{
  trapezoid_adjusted_rate = current_block->initial_rate;
  min_safe_rate = current_block->rate_delta + (current_block->rate_delta >> 1); // 1.5 x rate_delta
  trapezoid_tick_cycle_counter = CYCLES_PER_ACCELERATION_TICK/2; // Start halfway for midpoint rule.
  set_step_events_per_minute(trapezoid_adjusted_rate); // Initialize cycles_per_step_event
}

#if 0
// This function determines an acceleration velocity change every CYCLES_PER_ACCELERATION_TICK by
// keeping track of the number of elapsed cycles during a de/ac-celeration. The code assumes that 
// step_events occur significantly more often than the acceleration velocity iterations.
static uint8_t iterate_trapezoid_cycle_counter() 
{
  trapezoid_tick_cycle_counter += cycles_per_step_event;  
  if(trapezoid_tick_cycle_counter > CYCLES_PER_ACCELERATION_TICK) 
  {
    trapezoid_tick_cycle_counter -= CYCLES_PER_ACCELERATION_TICK;
    return(true);
  } 
  else 
  {
    return(false);
  }
}          
#endif

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. It is  executed at the rate set with
// config_step_timer. It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
// It is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port after each pulse. 
// The bresenham line tracer algorithm controls all three stepper outputs simultaneously with these two interrupts.
//SIGNAL(TIMER1_COMPA_vect)
void st_interrupt (void)
{        
  // TODO: Check if the busy-flag can be eliminated by just disabeling this interrupt while we are in it
  
  if(busy){ return; } // The busy-flag is used to avoid reentering this interrupt
  
  // Set the direction pins a cuple of nanoseconds before we step the steppers
  //STEPPING_PORT = (STEPPING_PORT & ~DIRECTION_MASK) | (out_bits & DIRECTION_MASK);
//  set_direction_pins (out_bits);
  
  // Then pulse the stepping pins
  //STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | out_bits;
#ifdef STEP_LED_NONE
  set_step_pins ();
#endif  
  
  // Reset step pulse reset timer so that The Stepper Port Reset Interrupt can reset the signal after
  // exactly settings.pulse_microseconds microseconds.
  //TODO
  //TCNT2 = -(((settings.pulse_microseconds-2)*TICKS_PER_MICROSECOND)/8);

  busy = true;
  sei(); // Re enable interrupts (normally disabled while inside an interrupt handler)
         // ((We re-enable interrupts in order for SIG_OVERFLOW2 to be able to be triggered 
         // at exactly the right time even if we occasionally spend a lot of time inside this handler.))
    
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block != NULL) {
      trapezoid_generator_reset();
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      counter_z = counter_x;
      counter_e = counter_x;
      step_events_completed = 0;     
      
      direction_bits = current_block->direction_bits;
      set_direction_pins ();
      step_bits_xyz = step_bits_e = 0;
    } else {
      st_go_idle();
    }    
  } 


  if (current_block != NULL) 
  {
    if (current_block->action_type == AT_MOVE)
    {
      // Execute step displacement profile by bresenham line algorithm
      step_bits_xyz = step_bits_e = 0;
      counter_x += current_block->steps_x;
      if (counter_x > 0) {
        step_bits_xyz |= (1<<X_STEP_BIT);
        counter_x -= current_block->step_event_count;
      }
      counter_y += current_block->steps_y;
      if (counter_y > 0) {
        step_bits_xyz |= (1<<Y_STEP_BIT);
        counter_y -= current_block->step_event_count;
      }
      counter_z += current_block->steps_z;
      if (counter_z > 0) {
        step_bits_xyz |= (1<<Z_STEP_BIT);
        counter_z -= current_block->step_event_count;
      }
      
      counter_e += current_block->steps_e;
      if (counter_e > 0) {
        step_bits_e |= (1<<E_STEP_BIT);
        counter_e -= current_block->step_event_count;
      }

#ifndef STEP_LED_NONE
      clear_step_pins ();
#endif

      step_events_completed++; // Iterate step events

      if (current_block->check_endstops)
      {
        if ( (current_block->steps_x && hit_home_stop_x (direction_bits & (1<<X_DIRECTION_BIT)) ) ||
             (current_block->steps_y && hit_home_stop_y (direction_bits & (1<<Y_DIRECTION_BIT)) ) ||
             (current_block->steps_z && hit_home_stop_z (direction_bits & (1<<Z_DIRECTION_BIT)) )
           )
        {
          step_events_completed = current_block->step_event_count;
          step_bits_xyz = step_bits_e = 0;
        }
      }
      
#ifndef STEP_LED_NONE
      set_step_pins ();
#endif

      // While in block steps, check for de/ac-celeration events and execute them accordingly.
      if (step_events_completed < current_block->step_event_count) {
        
        // The trapezoid generator always checks step event location to ensure de/ac-celerations are 
        // executed and terminated at exactly the right time. This helps prevent over/under-shooting
        // the target position and speed. 
        
        // NOTE: By increasing the ACCELERATION_TICKS_PER_SECOND in config.h, the resolution of the 
        // discrete velocity changes increase and accuracy can increase as well to a point. Numerical 
        // round-off errors can effect this, if set too high. This is important to note if a user has 
        // very high acceleration and/or feedrate requirements for their machine.
        
        if (step_events_completed < current_block->accelerate_until) 
        {
          // check if speeds need to be increased.
          if ( accel_flag ) {
            trapezoid_adjusted_rate += current_block->rate_delta;
            if (trapezoid_adjusted_rate >= current_block->nominal_rate) {
              // Reached nominal rate a little early. Cruise at nominal rate until decelerate_after.
              trapezoid_adjusted_rate = current_block->nominal_rate;
            }
            set_step_events_per_minute(trapezoid_adjusted_rate);
            accel_flag = false;
          }
        } 
        else if (step_events_completed >= current_block->decelerate_after) {
          // Reset trapezoid tick cycle counter to make sure that the deceleration is performed the
          // same every time. Reset to CYCLES_PER_ACCELERATION_TICK/2 to follow the midpoint rule for
          // an accurate approximation of the deceleration curve.
          if (step_events_completed == current_block-> decelerate_after) {
            trapezoid_tick_cycle_counter = CYCLES_PER_ACCELERATION_TICK/2;
          } else {
            // check if speeds need to be reduced.
            if ( accel_flag ) {  
              // NOTE: We will only reduce speed if the result will be > 0. This catches small
              // rounding errors that might leave steps hanging after the last trapezoid tick.
              if (trapezoid_adjusted_rate > current_block->rate_delta) {
                trapezoid_adjusted_rate -= current_block->rate_delta;
              }
              if (trapezoid_adjusted_rate < current_block->final_rate) {
                // Reached final rate a little early. Cruise to end of block at final rate.
                trapezoid_adjusted_rate = current_block->final_rate;
              }
              set_step_events_per_minute(trapezoid_adjusted_rate);
              accel_flag = 0;
            }
          }
        } else {
          // No accelerations. Make sure we cruise exactly at the nominal rate.
          if (trapezoid_adjusted_rate != current_block->nominal_rate) {
            trapezoid_adjusted_rate = current_block->nominal_rate;
            set_step_events_per_minute(trapezoid_adjusted_rate);
          }
        }
              
      } else {   
        // If current block is finished, reset pointer 
        current_block = NULL;
        plan_discard_current_block();
      }
    }
    else if (current_block->action_type == AT_WAIT_TEMPS) 
    {
      step_bits_xyz = step_bits_e = 0;
      if (temp_achieved(EXTRUDER_0))
      {
        current_block = NULL;
        plan_discard_current_block();
      }
    }
    else if (current_block->action_type == AT_WAIT)
    // Wait for all temperatures to rise to set value
    {
      step_bits_xyz = step_bits_e = 0;
      if (temps_achieved ())
      {
        current_block = NULL;
        plan_discard_current_block();
      }
    }
  } 
  else 
  {
    // Still no block? Set the stepper pins to low before sleeping.
    step_bits_xyz = step_bits_e = 0;
  }          
  
#ifdef STEP_LED_NONE
  clear_all_step_pins ();
#else
  if (current_block == NULL)
  {
    leds_enabled = 0;
    led_on = 0x00;
    clear_all_step_pins();
  }
  else
    clear_step_pins_by_state ();
#endif  
//  out_bits ^= settings.invert_mask;  // Apply stepper invert mask    
  busy=false;
}

// This interrupt is set up by SIG_OUTPUT_COMPARE1A when it sets the motor port bits. It resets
// the motor port after a short period (settings.pulse_microseconds) completing one step cycle.
//SIGNAL(TIMER2_OVF_vect)
void st_reset_interrupt (void)
{
  // reset stepping pins (leave the direction pins)
  // STEPPING_PORT = (STEPPING_PORT & ~STEP_MASK) | (settings.invert_mask & STEP_MASK); 
  clear_all_step_pins ();
}

void accelCallback (tHwTimer *pTimer, uint32_t int_mask)
{
  (void)pTimer;
  accel_flag = 1;
}

void stepCallback (tHwTimer *pTimer, uint32_t int_mask)
{
  (void)pTimer;

  digital_write (1, (1<<15), 1);

//  if (int_mask & _BIT(TIM_MR0_INT))
  {
    // call stepper function
    st_interrupt();
  }
  
  digital_write (1, (1<<15), 0);
}

static void init_dac (void)
{
	PINSEL_CFG_Type PinCfg;

	/*
	 * Init DAC pin connect
	 * AOUT on P0.26
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 26;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	/* init DAC structure to default
	 * Maximum	current is 700 uA
	 * First value to AOUT is 0
	 */
	DAC_Init(LPC_DAC);
}

// Initialize and start the stepper motor subsystem
void st_init()
{
#if 0
	// Configure directions of interface pins
  STEPPING_DDR   |= STEPPING_MASK;
  STEPPING_PORT = (STEPPING_PORT & ~STEPPING_MASK) | settings.invert_mask;
  STEPPERS_DISABLE_DDR |= 1<<STEPPERS_DISABLE_BIT;
  
	// waveform generation = 0100 = CTC
	TCCR1B &= ~(1<<WGM13);
	TCCR1B |=  (1<<WGM12);
	TCCR1A &= ~(1<<WGM11); 
	TCCR1A &= ~(1<<WGM10);

	// output mode = 00 (disconnected)
	TCCR1A &= ~(3<<COM1A0); 
	TCCR1A &= ~(3<<COM1B0); 
	
	// Configure Timer 2
  TCCR2A = 0;         // Normal operation
  TCCR2B = (1<<CS21); // Full speed, 1/8 prescaler
  TIMSK2 |= (1<<TOIE2);      
#else
  init_dac();
  
  // set up timers
  // use hardware timer 0 and 1
  setupHwTimer(1, stepCallback);

  setupHwTimer(0, accelCallback);
  setHwTimerInterval (0, TICKS_PER_MICROSECOND*1000);
  enableHwTimer(0);


#if 0
  // set the LED blink times, 50 ms on/off = 10 flashes per second
  led_on_time = 50;
  led_off_time = 50;

  AddSlowTimer (&blinkTimer);
#endif

  // setup debug pin
  pin_mode(1, (1 << 15), OUTPUT);
    
#endif
  
  set_step_events_per_minute(6000);
  trapezoid_tick_cycle_counter = 0;
  
  // Start in the idle state
  st_go_idle();
}

// Block until all buffered steps are executed
void st_synchronize()
{
  while(plan_get_current_block()) { sleep_mode(); }    
}

// Configures the prescaler and ceiling of timer 1 to produce the given rate as accurately as possible.
// Returns the actual number of cycles per interrupt
static uint32_t config_step_timer(uint32_t cycles)
{
#if 0
  uint16_t ceiling;
  uint16_t prescaler;
  uint32_t actual_cycles;
	if (cycles <= 0xffffL) {
		ceiling = cycles;
    prescaler = 0; // prescaler: 0
    actual_cycles = ceiling;
	} else if (cycles <= 0x7ffffL) {
    ceiling = cycles >> 3;
    prescaler = 1; // prescaler: 8
    actual_cycles = ceiling * 8L;
	} else if (cycles <= 0x3fffffL) {
		ceiling =  cycles >> 6;
    prescaler = 2; // prescaler: 64
    actual_cycles = ceiling * 64L;
	} else if (cycles <= 0xffffffL) {
		ceiling =  (cycles >> 8);
    prescaler = 3; // prescaler: 256
    actual_cycles = ceiling * 256L;
	} else if (cycles <= 0x3ffffffL) {
		ceiling = (cycles >> 10);
    prescaler = 4; // prescaler: 1024
    actual_cycles = ceiling * 1024L;    
	} else {
	  // Okay, that was slower than we actually go. Just set the slowest speed
		ceiling = 0xffff;
    prescaler = 4;
    actual_cycles = 0xffff * 1024;
	}
	// Set prescaler
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | ((prescaler+1)<<CS10);
  // Set ceiling
  OCR1A = ceiling;
  return(actual_cycles);
#else
  if (cycles > 0x3ffffffL)
    cycles = 0x4000000;
    
  setHwTimerInterval (1, cycles);

  return cycles;
#endif
}

static void set_step_events_per_minute(uint32_t steps_per_minute) 
{
  
  if (steps_per_minute < MINIMUM_STEPS_PER_MINUTE) 
  { 
    steps_per_minute = MINIMUM_STEPS_PER_MINUTE; 
  }
  cycles_per_step_event = config_step_timer((TICKS_PER_MICROSECOND*1000000*6)/steps_per_minute*10);
  
  DAC_UpdateValue (LPC_DAC, steps_per_minute/dac_scale);
}

#if 0
// not used
void st_go_home()
{
  limits_go_home();  
  plan_set_current_position_xyz(0,0,0);
}
#endif

