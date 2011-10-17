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

static uint8_t led_count [NUM_AXES];
static uint8_t led_on;


// Locals

static block_t *current_block;  // A pointer to the block currently being traced

// Variables used by The Stepper Driver Interrupt
static uint8_t out_bits;        // The next stepping-bits to be output
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

static void  set_direction_pins (uint8_t bits) 
{
    x_direction( (bits & (1<<X_DIRECTION_BIT))?0:1);
    y_direction( (bits & (1<<Y_DIRECTION_BIT))?0:1);
    z_direction( (bits & (1<<Z_DIRECTION_BIT))?0:1);
    e_direction( (bits & (1<<E_DIRECTION_BIT))?0:1);
}

static void  set_step_pins (uint8_t bits) 
{
  if (bits & (1<<X_STEP_BIT))
  {
    x_step();
    inc_led_count (&led_count[X_AXIS], (1<<X_STEP_BIT));
  }
  if (bits & (1<<Y_STEP_BIT))
  {
    y_step();
    inc_led_count (&led_count[Y_AXIS], (1<<Y_STEP_BIT));
  }  
  if (bits & (1<<Z_STEP_BIT))
  {
    z_step();
    inc_led_count (&led_count[Z_AXIS], (1<<Z_STEP_BIT));
  }
  if (bits & (1<<E_STEP_BIT))
  {
    e_step();
    inc_led_count (&led_count[E_AXIS], (1<<E_STEP_BIT));
  }
}

static void  clear_step_pins (uint8_t bits) 
{
  if (bits & (1<<X_STEP_BIT))    x_unstep();
  if (bits & (1<<Y_STEP_BIT))    y_unstep();
  if (bits & (1<<Z_STEP_BIT))    z_unstep();
  if (bits & (1<<E_STEP_BIT))    e_unstep();
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
//  digital_write (1, (1<<15), 1);

  enableHwTimer(1);
  
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
//  digital_write (1, (1<<15), 0);

  disableHwTimer(1);
  clear_step_pins(0x0F);
#endif
}

// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
static void trapezoid_generator_reset() {
  trapezoid_adjusted_rate = current_block->initial_rate;
  min_safe_rate = current_block->rate_delta + (current_block->rate_delta >> 1); // 1.5 x rate_delta
  trapezoid_tick_cycle_counter = CYCLES_PER_ACCELERATION_TICK/2; // Start halfway for midpoint rule.
  set_step_events_per_minute(trapezoid_adjusted_rate); // Initialize cycles_per_step_event
}

// This function determines an acceleration velocity change every CYCLES_PER_ACCELERATION_TICK by
// keeping track of the number of elapsed cycles during a de/ac-celeration. The code assumes that 
// step_events occur significantly more often than the acceleration velocity iterations.
static uint8_t iterate_trapezoid_cycle_counter() {
  trapezoid_tick_cycle_counter += cycles_per_step_event;  
  if(trapezoid_tick_cycle_counter > CYCLES_PER_ACCELERATION_TICK) {
    trapezoid_tick_cycle_counter -= CYCLES_PER_ACCELERATION_TICK;
    return(true);
  } else {
    return(false);
  }
}          

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
//!  set_step_pins (out_bits);
  
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
      
      out_bits = current_block->direction_bits;
      set_direction_pins (out_bits);
    } else {
      st_go_idle();
    }    
  } 

  if (current_block != NULL) 
  {
    if (current_block->action_type == AT_MOVE)
    {
      // Execute step displacement profile by bresenham line algorithm
      out_bits = current_block->direction_bits;
      counter_x += current_block->steps_x;
      if (counter_x > 0) {
        out_bits |= (1<<X_STEP_BIT);
        counter_x -= current_block->step_event_count;
      }
      counter_y += current_block->steps_y;
      if (counter_y > 0) {
        out_bits |= (1<<Y_STEP_BIT);
        counter_y -= current_block->step_event_count;
      }
      counter_z += current_block->steps_z;
      if (counter_z > 0) {
        out_bits |= (1<<Z_STEP_BIT);
        counter_z -= current_block->step_event_count;
      }
      
      counter_e += current_block->steps_e;
      if (counter_e > 0) {
        out_bits |= (1<<E_STEP_BIT);
        counter_e -= current_block->step_event_count;
      }

      step_events_completed++; // Iterate step events

      if (current_block->check_endstops)
      {
        if ( (current_block->steps_x && hit_home_stop_x (out_bits & (1<<X_DIRECTION_BIT)) ) ||
             (current_block->steps_y && hit_home_stop_y (out_bits & (1<<Y_DIRECTION_BIT)) ) ||
             (current_block->steps_z && hit_home_stop_z (out_bits & (1<<Z_DIRECTION_BIT)) )
           )
        {
          step_events_completed = current_block->step_event_count;
          out_bits = 0;
        }
      }
      
      // While in block steps, check for de/ac-celeration events and execute them accordingly.
      if (step_events_completed < current_block->step_event_count) {
        
        // The trapezoid generator always checks step event location to ensure de/ac-celerations are 
        // executed and terminated at exactly the right time. This helps prevent over/under-shooting
        // the target position and speed. 
        
        // NOTE: By increasing the ACCELERATION_TICKS_PER_SECOND in config.h, the resolution of the 
        // discrete velocity changes increase and accuracy can increase as well to a point. Numerical 
        // round-off errors can effect this, if set too high. This is important to note if a user has 
        // very high acceleration and/or feedrate requirements for their machine.
        
        if (step_events_completed < current_block->accelerate_until) {
          // Iterate cycle counter and check if speeds need to be increased.
          if ( iterate_trapezoid_cycle_counter() ) {
            trapezoid_adjusted_rate += current_block->rate_delta;
            if (trapezoid_adjusted_rate >= current_block->nominal_rate) {
              // Reached nominal rate a little early. Cruise at nominal rate until decelerate_after.
              trapezoid_adjusted_rate = current_block->nominal_rate;
            }
            set_step_events_per_minute(trapezoid_adjusted_rate);
          }
        } else if (step_events_completed >= current_block->decelerate_after) {
          // Reset trapezoid tick cycle counter to make sure that the deceleration is performed the
          // same every time. Reset to CYCLES_PER_ACCELERATION_TICK/2 to follow the midpoint rule for
          // an accurate approximation of the deceleration curve.
          if (step_events_completed == current_block-> decelerate_after) {
            trapezoid_tick_cycle_counter = CYCLES_PER_ACCELERATION_TICK/2;
          } else {
            // Iterate cycle counter and check if speeds need to be reduced.
            if ( iterate_trapezoid_cycle_counter() ) {  
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
      out_bits = 0;
      if (temp_achieved(EXTRUDER_0))
      {
        current_block = NULL;
        plan_discard_current_block();
      }
    }  
  } 
  else 
  {
    // Still no block? Set the stepper pins to low before sleeping.
    out_bits = 0;
  }          
  
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
  clear_step_pins (ALL_STEP_PINS);
}

void stepCallback (tHwTimer *pTimer, uint32_t int_mask)
{
  (void)pTimer;

  digital_write (1, (1<<15), 1);

  if (int_mask & _BIT(TIM_MR0_INT))
  {
    // decide which outputs need stepping
    st_interrupt();
    clear_step_pins (out_bits);
  }

  if (int_mask & _BIT(TIM_MR1_INT))
  { 
    // step the required channels
    set_step_pins (out_bits);
      
    //st_reset_interrupt();
  }
  
  if (int_mask & _BIT(TIM_MR2_INT))
  { 
#ifdef STEP_LED_NONE
    // turn off all step outputs
    clear_step_pins (ALL_STEP_PINS);
#elif !defined(STEP_LED_ON_WHEN_ACTIVE)
    // turn off step outputs
    if ((led_on & (1<<X_STEP_BIT)) == 0)
    {
      x_unstep();
    }
    if ((led_on & (1<<Y_STEP_BIT)) == 0)
    {
      y_unstep();
    }
    if ((led_on & (1<<Z_STEP_BIT)) == 0)
    {
      z_unstep();
    }
    if ((led_on & (1<<E_STEP_BIT)) == 0)
    {
      e_unstep();
    }
    // else leave as is (important!)
#endif
  }
  
  digital_write (1, (1<<15), 0);
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
  // set up timers
  // we use hardware timer 1
  setupHwTimer(1, stepCallback);

  // Set the Match 1 and Match 2 interrupts
  // The time from Match0 to Match 1 defines the low pulse period of the step output
  // and the time from Match1 to Match 2 defines the minimum high pulse period of the step output-
  // if the LED is in a blink ON period the step output will be left high until next required step
  setHwTimerMatch(1, 1, 1000); // Match1 about Match0 + 5 us
  setHwTimerMatch(1, 2, 1500); // Match2, about Match0 + 10 us
  
  //debug
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

static void set_step_events_per_minute(uint32_t steps_per_minute) {
  if (steps_per_minute < MINIMUM_STEPS_PER_MINUTE) { steps_per_minute = MINIMUM_STEPS_PER_MINUTE; }
  cycles_per_step_event = config_step_timer((TICKS_PER_MICROSECOND*1000000*6)/steps_per_minute*10);
}

#if 0
// not used
void st_go_home()
{
  limits_go_home();  
  plan_set_current_position_xyz(0,0,0);
}
#endif

