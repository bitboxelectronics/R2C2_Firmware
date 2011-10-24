/*
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#ifndef stepper_h
#define stepper_h 

//#include <avr/io.h>
//#include <avr/sleep.h>

#include "pinout.h"

// from nuts_bolts.h:
#define square(x) ((x)*(x))
#define sleep_mode(x) do {} while (0)
#define sei(x) 

#define NUM_AXES 4

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3

#define clear_vector(a) memset(a, 0, sizeof(a))
#define clear_vector_double(a) memset(a, 0.0, sizeof(a))
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
// end

// From grbl/config.h


#define X_STEP_BIT    20           
#define Y_STEP_BIT    25       
#define Z_STEP_BIT    29       
#define E_STEP_BIT    10       

#define X_DIRECTION_BIT   23     
#define Y_DIRECTION_BIT   26   
#define Z_DIRECTION_BIT   0   
#define E_DIRECTION_BIT   11   

// This parameter sets the delay time before disabling the steppers after the final block of movement.
// A short delay ensures the steppers come to a complete stop and the residual inertial force in the 
// CNC axes don't cause the axes to drift off position. This is particularly important when manually 
// entering g-code into grbl, i.e. locating part zero or simple manual machining. If the axes drift,
// grbl has no way to know this has happened, since stepper motors are open-loop control. Depending
// on the machine, this parameter may need to be larger or smaller than the default time.
#define STEPPER_IDLE_LOCK_TIME 25 // (milliseconds)

// The temporal resolution of the acceleration management subsystem. Higher number give smoother
// acceleration but may impact performance.
// NOTE: Increasing this parameter will help any resolution related issues, especially with machines 
// requiring very high accelerations and/or very fast feedrates. In general, this will reduce the 
// error between how the planner plans the motions and how the stepper program actually performs them.
// However, at some point, the resolution can be high enough, where the errors related to numerical 
// round-off can be great enough to cause problems and/or it's too fast for the Arduino. The correct
// value for this parameter is machine dependent, so it's advised to set this only as high as needed.
// Approximate successful values can range from 30L to 100L or more.
#define ACCELERATION_TICKS_PER_SECOND 1000L

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.0 // (mm/min)

// Minimum stepper rate. Sets the absolute minimum stepper rate in the stepper program and never run
// slower than this value, except when sleeping. This parameter overrides the minimum planner speed.
// This is primarily used to guarantee that the end of a movement is always reached and not stop to
// never reach its target. This parameter should always be greater than zero.
#define MINIMUM_STEPS_PER_MINUTE 1200 // (steps/min) - Integer value only

// Number of arc generation iterations by small angle approximation before exact arc
// trajectory correction. Value must be 1-255. This parameter maybe decreased if there are issues
// with the accuracy of the arc generations. In general, the default value is more than enough for
// the intended CNC applications of grbl, and should be on the order or greater than the size of
// the buffer to help with the computational efficiency of generating arcs.
#define N_ARC_CORRECTION 25 

// end

#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits
#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
#define STEPPING_MASK (STEP_MASK | DIRECTION_MASK) // All stepping-related bits (step/direction)

// Initialize and start the stepper motor subsystem
void st_init();

// Block until all buffered steps are executed
void st_synchronize();

// Execute the homing cycle
void st_go_home();
             
// The stepper subsystem goes to sleep when it runs out of things to execute. Call this
// to notify the subsystem that it is time to go to work.
void st_wake_up();

// steptimeout is set to zero when we step, and increases over time so we can turn the motors off when they've been idle for a while
extern volatile uint16_t steptimeout;

#endif
