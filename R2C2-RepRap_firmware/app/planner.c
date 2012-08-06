/*
  planner.c - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Sungeun K. Jeon
  
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

/* The ring buffer implementation gleaned from the wiring_serial library by David A. Mellis. */

#include <inttypes.h>
#include <stdbool.h>
#include <math.h>       
#include <stdlib.h>
#include <string.h>

#include "r2c2.h"

#include "planner.h"
//#include "nuts_bolts.h"
#include "stepper.h"
//#include "settings.h"
//#include "config.h"

#include "app_config.h"

// The number of linear motions that can be in the plan at any give time (+1)
// NB: The actual maximum number of active entries is the size - 1
#define BLOCK_BUFFER_SIZE 20

tTarget startpoint;

static block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions
static volatile uint8_t block_buffer_head;       // Index of the next block to process
static volatile uint8_t block_buffer_tail;       // Index of the last block

static int32_t position[NUM_AXES];             // The current position of the tool in absolute steps
static double previous_unit_vec[NUM_AXES];     // Unit vector of previous path line segment
static double previous_nominal_speed;   // Nominal speed of previous path line segment

static uint8_t acceleration_manager_enabled;   // Acceleration management active?

#ifdef _CROSSWORKS
long lround (double x)
{
    return x+0.5;
}
#endif

// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.
static int8_t next_block_index(int8_t block_index) {
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) 
    { block_index = 0; }
  return(block_index);
}


// Returns the index of the previous block in the ring buffer
static int8_t prev_block_index(int8_t block_index) {
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}


// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:
static double estimate_acceleration_distance(double initial_rate, double target_rate, double acceleration) {
  return( (target_rate*target_rate-initial_rate*initial_rate)/(2*acceleration) );
}


/*                        + <- some maximum rate we don't care about
                         /|\
                        / | \                    
                       /  |  + <- final_rate     
                      /   |  |                   
     initial_rate -> +----+--+                   
                          ^  ^                   
                          |  |                   
      intersection_distance  distance                                                                           */
// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)
static double intersection_distance(double initial_rate, double final_rate, double acceleration, double distance) {
  return( (2*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/(4*acceleration) );
}

            
// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity
// using the acceleration within the allotted distance.
// NOTE: sqrt() reimplemented here from prior version due to improved planner logic. Increases speed
// in time critical computations, i.e. arcs or rapid short lines from curves. Guaranteed to not exceed
// BLOCK_BUFFER_SIZE calls per planner cycle.
static double max_allowable_speed(double acceleration, double target_velocity, double distance) 
{
  return( sqrt(target_velocity*target_velocity-2*acceleration*60*60*distance) );
}


// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
static bool planner_reverse_pass_kernel(block_t *prev, block_t *current, double next_entry_speed) 
{
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and 
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (current->entry_speed != current->max_entry_speed) 
	{    
      // If nominal length true, max junction speed is guaranteed to be reached. Only compute
      // for max allowable speed if block is decelerating and nominal length is false.
    if ((!current->nominal_length_flag) && (current->max_entry_speed > next_entry_speed)) 
	  {
      current->entry_speed = min( current->max_entry_speed, max_allowable_speed(-config.acceleration, next_entry_speed, current->millimeters));
      } else 
	  {
        current->entry_speed = current->max_entry_speed;
      } 
      current->recalculate_flag = true;    
    return true;
    }

  return false;
}


// This implements the reverse pass.
static void planner_reverse_pass() 
{
  uint8_t block_index = block_buffer_tail;
  block_t *prev, *cur, *next;
  bool replan_prev = false;

  if (plan_queue_size() <= 2)
    return;

  next = NULL;
  block_index = prev_block_index( block_index );
  cur = &block_buffer[block_index];
  block_index = prev_block_index( block_index );
  prev = &block_buffer[block_index];

  replan_prev = planner_reverse_pass_kernel (prev, cur, 0.0);

  // move pointers
  block_index = prev_block_index( block_index );
  next = cur;
  cur = prev;
  prev = &block_buffer[block_index];

  while(block_index != block_buffer_head) 
  {    
    if (replan_prev && cur)
      cur->recalculate_flag = true;

    // Skip buffer head/first block to prevent over-writing the initial entry speed.
    if ( (cur != NULL) && (next != NULL) )
      replan_prev = planner_reverse_pass_kernel (prev, cur, next->entry_speed);

    // move pointers
    block_index = prev_block_index( block_index );
  	next = cur;
    cur = prev;
    prev = &block_buffer[block_index];

  }

    if (replan_prev && cur)
      cur->recalculate_flag = true;
}


// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
static void planner_forward_pass_kernel(block_t *previous, block_t *current) 
{
  // Begin planning after buffer_head
  
  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.  
  if (!previous->nominal_length_flag) 
  {
    if (previous->entry_speed < current->entry_speed) 
	{
      double entry_speed = min( current->entry_speed,
        max_allowable_speed(-config.acceleration,previous->entry_speed,previous->millimeters) );

      // Check for junction speed change
      if (current->entry_speed != entry_speed) 
	  {
        current->entry_speed = entry_speed;
        current->recalculate_flag = true;
      }
    }    
  }
}


// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the forward pass.
static void planner_forward_pass() 
{
  int8_t block_index = block_buffer_head;
  block_t *prev, *cur, *next;

  prev = cur = next = NULL;
  
  while(block_index != block_buffer_tail) 
  {
    prev = cur;
    cur = next;
    next = &block_buffer[block_index];

	if ( (cur != NULL) && (prev != NULL) )
		planner_forward_pass_kernel (prev, cur);
  
    block_index = next_block_index( block_index );
  }

  if (cur != NULL)
	planner_forward_pass_kernel(cur, next);
}


/*                             STEPPER RATE DEFINITION                                              
                                     +--------+   <- nominal_rate
                                    /          \                                
    nominal_rate*entry_factor ->   +            \                               
                                   |             + <- nominal_rate*exit_factor  
                                   +-------------+                              
                                       time -->                                 
*/                                                                              
// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
// The factors represent a factor of braking and must be in the range 0.0-1.0.
// This converts the planner parameters to the data required by the stepper controller.
// NOTE: Final rates must be computed in terms of their respective blocks.
static void calculate_trapezoid_for_block(block_t *block, double entry_factor, double exit_factor) {
  
  int32_t acceleration_per_minute;
  int32_t accelerate_steps;
  int32_t decelerate_steps;
  int32_t plateau_steps;

  block->initial_rate = ceil(block->nominal_rate*entry_factor); // (step/min)
  block->final_rate = ceil(block->nominal_rate*exit_factor); // (step/min)
  acceleration_per_minute = block->rate_delta*ACCELERATION_TICKS_PER_SECOND*60.0; // (step/min^2)
  accelerate_steps = ceil(estimate_acceleration_distance(block->initial_rate, block->nominal_rate, acceleration_per_minute));
  decelerate_steps = floor(estimate_acceleration_distance(block->nominal_rate, block->final_rate, -acceleration_per_minute));

  // Calculate the size of Plateau of Nominal Rate. 
  plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;
  
  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort acceleration and start braking 
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) 
  {  
    accelerate_steps = ceil(intersection_distance(block->initial_rate, block->final_rate, acceleration_per_minute, block->step_event_count));
    accelerate_steps = max(accelerate_steps,0); // Check limits due to numerical round-off
    accelerate_steps = min(accelerate_steps,block->step_event_count);
    plateau_steps = 0;
  }  
  
  block->accelerate_until = accelerate_steps;
  block->decelerate_after = accelerate_steps+plateau_steps;
  block->recalculate_flag = false;
}     

/*                            PLANNER SPEED DEFINITION                                              
                                     +--------+   <- current->nominal_speed
                                    /          \                                
         current->entry_speed ->   +            \                               
                                   |             + <- next->entry_speed
                                   +-------------+                              
                                       time -->                                 
*/                                                                              
// Recalculates the trapezoid speed profiles for flagged blocks in the plan according to the 
// entry_speed for each junction and the entry_speed of the next junction. Must be called by 
// planner_recalculate() after updating the blocks. Any recalulate flagged junction will
// compute the two adjacent trapezoids to the junction, since the junction speed corresponds 
// to exit speed and entry speed of one another.
static void planner_recalculate_trapezoids() 
{
  int8_t block_index = block_buffer_head;
  block_t *current;
  block_t *next = NULL;
  
  while(block_index != block_buffer_tail) 
  {
    current = next;
    next = &block_buffer[block_index];
    
    if (current) 
    {
      // Recalculate if current block entry or exit junction speed has changed.
      //if (current->recalculate_flag || next->recalculate_flag) 
      if (current->recalculate_flag)
      {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.     
        calculate_trapezoid_for_block(current, current->entry_speed/current->nominal_speed,
          next->entry_speed/current->nominal_speed);      
      }
    }
    block_index = next_block_index( block_index );
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if (next->recalculate_flag)
  {
    calculate_trapezoid_for_block(next, next->entry_speed/next->nominal_speed, MINIMUM_PLANNER_SPEED/next->nominal_speed);
  }
}

// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_speed) 
//      so that:
//     a. The junction speed is equal to or less than the maximum junction speed limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant 
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed values if 
//     a. The speed increase within one block would require faster acceleration than the one, true 
//        constant acceleration.
//
// When these stages are complete all blocks have an entry speed that will allow all speed changes to 
// be performed using only the one, true constant acceleration, and where no junction speed is greater
// than the max limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks using the recently updated junction speeds. Block trapezoids
//      with no updated junction speeds will not be recalculated and assumed ok as is.
//
// All planner computations are performed with doubles (float on Arduinos) to minimize numerical round-
// off errors. Only when planned values are converted to stepper rate parameters, these are integers.

static void planner_recalculate() {     
  planner_reverse_pass();
  planner_forward_pass();
  
  planner_recalculate_trapezoids();
}

void plan_init() {
  block_buffer_head = 0;
  block_buffer_tail = 0;
  plan_set_acceleration_manager_enabled(true);
  clear_vector(position);
  clear_vector_double(previous_unit_vec);
  previous_nominal_speed = 0.0;
  
  memset (&startpoint, 0, sizeof(startpoint));
}

void plan_set_acceleration_manager_enabled(uint8_t enabled) {
  if ((!!acceleration_manager_enabled) != (!!enabled)) {
    st_synchronize();
    acceleration_manager_enabled = !!enabled;
  }
}

int plan_is_acceleration_manager_enabled() {
  return(acceleration_manager_enabled);
}

void plan_discard_current_block() {
  if (block_buffer_head != block_buffer_tail) {
    block_buffer_head = next_block_index( block_buffer_head );
  }
}

block_t *plan_get_current_block() {
  if (block_buffer_head == block_buffer_tail) { return(NULL); }
  return(&block_buffer[block_buffer_head]);
}

double calc_inverse_minute (bool invert_feed_rate, double feed_rate, double inverse_millimeters)
{
  if (!invert_feed_rate) {
    return feed_rate * inverse_millimeters;
  } else {
    return 1.0 / feed_rate;
  }
}

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in 
// millimaters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
//void plan_buffer_line(double x, double y, double z, double feed_rate, uint8_t invert_feed_rate)
void plan_buffer_line (tActionRequest *pAction)
{
  double x;
  double y;
  double z;
  double feed_rate;
  uint8_t invert_feed_rate;
  bool e_only = false;
  double speed_x, speed_y, speed_z, speed_e; // Nominal mm/minute for each axis
  int32_t target[NUM_AXES];
  uint8_t next_buffer_tail;
  block_t *block;
  double delta_mm[NUM_AXES];
  double inverse_millimeters;
  double microseconds;
  double multiplier;
  double speed_factor;
  
  x = pAction->target.x;
  y = pAction->target.y;
  z = pAction->target.z;
  feed_rate = pAction->target.feed_rate;
  invert_feed_rate = pAction->target.invert_feed_rate;
  
  // Calculate target position in absolute steps
  target[X_AXIS] = lround(x*(double)config.axis[X_AXIS].steps_per_mm);
  target[Y_AXIS] = lround(y*(double)config.axis[Y_AXIS].steps_per_mm);
  target[Z_AXIS] = lround(z*(double)config.axis[Z_AXIS].steps_per_mm);     
  target[E_AXIS] = lround(pAction->target.e*(double)config.axis[E_AXIS].steps_per_mm);     
  
  // ----
  // Calculate the buffer tail after we push this block
  next_buffer_tail = next_block_index( block_buffer_tail );	
  
  // If the buffer is full: good! That means we are well ahead of the robot. 
  // Rest here until there is room in the buffer.
  while(block_buffer_head == next_buffer_tail) { sleep_mode(); }
  
  // Prepare to set up new block
  block = &block_buffer[block_buffer_tail];
  // ----

  block->action_type = AT_MOVE;
  
  // Compute direction bits for this block
  block->direction_bits = 0;
  if (target[X_AXIS] < position[X_AXIS]) { block->direction_bits |= _BV(X_AXIS); }
  if (target[Y_AXIS] < position[Y_AXIS]) { block->direction_bits |= _BV(Y_AXIS); }
  if (target[Z_AXIS] < position[Z_AXIS]) { block->direction_bits |= _BV(Z_AXIS); }
  if (target[E_AXIS] < position[E_AXIS]) { block->direction_bits |= _BV(E_AXIS); }
  
  // Number of steps for each axis
  block->steps_x = labs(target[X_AXIS]-position[X_AXIS]);
  block->steps_y = labs(target[Y_AXIS]-position[Y_AXIS]);
  block->steps_z = labs(target[Z_AXIS]-position[Z_AXIS]);
  block->steps_e = labs(target[E_AXIS]-position[E_AXIS]);
  block->step_event_count = max(block->steps_x, max(block->steps_y, block->steps_z));
  block->step_event_count = max(block->step_event_count, block->steps_e);

  // Bail if this is a zero-length block
  if (block->step_event_count == 0) { return; };
  
  // Compute path vector in terms of absolute step target and current positions
  delta_mm[X_AXIS] = (target[X_AXIS]-position[X_AXIS])/(double)config.axis[X_AXIS].steps_per_mm;
  delta_mm[Y_AXIS] = (target[Y_AXIS]-position[Y_AXIS])/(double)config.axis[Y_AXIS].steps_per_mm;
  delta_mm[Z_AXIS] = (target[Z_AXIS]-position[Z_AXIS])/(double)config.axis[Z_AXIS].steps_per_mm;
  delta_mm[E_AXIS] = (target[E_AXIS]-position[E_AXIS])/(double)config.axis[E_AXIS].steps_per_mm;
  block->millimeters = sqrt(square(delta_mm[X_AXIS]) + square(delta_mm[Y_AXIS]) + 
                            square(delta_mm[Z_AXIS]));
  if (block->millimeters == 0)
  {
    e_only = true;
    block->millimeters = fabs(delta_mm[E_AXIS]);
  }
  inverse_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple divides	
  
//
// Speed limit code from Marlin firmware
//
//TODO: handle invert_feed_rate
  //if(feedrate<minimumfeedrate)
  //  feedrate=minimumfeedrate;
  microseconds = lround((block->millimeters/feed_rate*60.0)*1000000.0);

  // Calculate speed in mm/minute for each axis
  multiplier = 60.0*1000000.0/(double)microseconds;
  speed_x = delta_mm[X_AXIS] * multiplier;
  speed_y = delta_mm[Y_AXIS] * multiplier;
  speed_z = delta_mm[Z_AXIS] * multiplier;
  speed_e = delta_mm[E_AXIS] * multiplier;

  // Limit speed per axis
  speed_factor = 1; //factor <=1 do decrease speed
  if(fabs(speed_x) > config.axis[X_AXIS].maximum_feedrate) 
  {
    speed_factor = (double)config.axis[X_AXIS].maximum_feedrate / fabs(speed_x);
  }
  if(fabs(speed_y) > config.axis[Y_AXIS].maximum_feedrate)
  {
    double tmp_speed_factor = (double)config.axis[Y_AXIS].maximum_feedrate / fabs(speed_y);
    if(speed_factor > tmp_speed_factor) speed_factor = tmp_speed_factor;
  }
  if(fabs(speed_z) > config.axis[Z_AXIS].maximum_feedrate)
  {
    double tmp_speed_factor = (double)config.axis[Z_AXIS].maximum_feedrate / fabs(speed_z);
    if(speed_factor > tmp_speed_factor) speed_factor = tmp_speed_factor;
  }
  if(fabs(speed_e) > config.axis[E_AXIS].maximum_feedrate)
  {
    double tmp_speed_factor = (double)config.axis[E_AXIS].maximum_feedrate / fabs(speed_e);
    if(speed_factor > tmp_speed_factor) speed_factor = tmp_speed_factor;
  }

  multiplier = multiplier * speed_factor;
  speed_x = delta_mm[X_AXIS] * multiplier;
  speed_y = delta_mm[Y_AXIS] * multiplier;
  speed_z = delta_mm[Z_AXIS] * multiplier;
  speed_e = delta_mm[E_AXIS] * multiplier;
  block->nominal_speed = block->millimeters * multiplier;    // mm per min
  block->nominal_rate = ceil(block->step_event_count * multiplier);   // steps per minute

//---  
#if 0
  // Calculate speed in mm/minute for each axis. No divide by zero due to previous checks.
  // NOTE: Minimum stepper speed is limited by MINIMUM_STEPS_PER_MINUTE in stepper.c
  double inverse_minute;
  if (!invert_feed_rate) {
    inverse_minute = feed_rate * inverse_millimeters;
  } else {
    inverse_minute = 1.0 / feed_rate;
  }
  block->nominal_speed = block->millimeters * inverse_minute; // (mm/min) Always > 0
  block->nominal_rate = ceil(block->step_event_count * inverse_minute); // (step/min) Always > 0
#endif
  
#if 0
  double axis_speed;
  axis_speed = delta_mm[Z_AXIS] * inverse_minute;
  if (axis_speed > config.axis[Z_AXIS].maximum_feedrate)
  {
    inverse_millimeters = 1.0 / delta_mm[Z_AXIS];
    inverse_minute = calc_inverse_minute (false, config.axis[Z_AXIS].maximum_feedrate, inverse_millimeters);
    
    block->nominal_speed = delta_mm[Z_AXIS] * inverse_minute; // (mm/min) Always > 0
    block->nominal_rate = ceil(block->step_event_count * inverse_minute); // (step/min) Always > 0
  }
#endif
  
  // Compute the acceleration rate for the trapezoid generator. Depending on the slope of the line
  // average travel per step event changes. For a line along one axis the travel per step event
  // is equal to the travel/step in the particular axis. For a 45 degree line the steppers of both
  // axes might step for every step event. Travel per step event is then sqrt(travel_x^2+travel_y^2).
  // To generate trapezoids with contant acceleration between blocks the rate_delta must be computed 
  // specifically for each line to compensate for this phenomenon:
  // Convert universal acceleration for direction-dependent stepper rate change parameter
  block->rate_delta = ceil( block->step_event_count*inverse_millimeters *  
        config.acceleration*60.0 / ACCELERATION_TICKS_PER_SECOND ); // (step/min/acceleration_tick)

#if 0
  double rate_calc;
  if (delta_mm[Z_AXIS] > 0)
  {
    rate_calc = ceil( block->step_event_count / delta_mm[Z_AXIS] *  
          50*60.0 / ACCELERATION_TICKS_PER_SECOND ); // (step/min/acceleration_tick)
    
    if (rate_calc < block->rate_delta)
      block->rate_delta = rate_calc;
  }
#endif    
  // Perform planner-enabled calculations
  if (acceleration_manager_enabled /*&& !e_only*/ ) {  
  
    // Compute path unit vector                            
    double unit_vec[NUM_AXES];
	double vmax_junction;
	double v_allowable;

    unit_vec[X_AXIS] = delta_mm[X_AXIS]*inverse_millimeters;
    unit_vec[Y_AXIS] = delta_mm[Y_AXIS]*inverse_millimeters;
    unit_vec[Z_AXIS] = delta_mm[Z_AXIS]*inverse_millimeters;  
  
    // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
    // Let a circle be tangent to both previous and current path line segments, where the junction 
    // deviation is defined as the distance from the junction to the closest edge of the circle, 
    // colinear with the circle center. The circular segment joining the two paths represents the 
    // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
    // radius of the circle, defined indirectly by junction deviation. This may be also viewed as 
    // path width or max_jerk in the previous grbl version. This approach does not actually deviate 
    // from path, but used as a robust way to compute cornering speeds, as it takes into account the
    // nonlinearities of both the junction angle and junction velocity.
    vmax_junction = MINIMUM_PLANNER_SPEED; // Set default max junction speed

    // Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
    if ((block_buffer_head != block_buffer_tail) && (previous_nominal_speed > 0.0)) 
    {
      // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
      // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
      double cos_theta = - previous_unit_vec[X_AXIS] * unit_vec[X_AXIS] 
                         - previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS] 
                         - previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] ;
                           
      // Skip and use default max junction speed for 0 degree acute junction.
      if (cos_theta < 0.95) 
      {
        vmax_junction = min(previous_nominal_speed,block->nominal_speed);
        // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
        if (cos_theta > -0.95) 
        {
          // Compute maximum junction velocity based on maximum acceleration and junction deviation
          double sin_theta_d2 = sqrt(0.5*(1.0-cos_theta)); // Trig half angle identity. Always positive.
          vmax_junction = min(vmax_junction,
            sqrt(config.acceleration*60*60 * config.junction_deviation * sin_theta_d2/(1.0-sin_theta_d2)) );
        }
      }
    }
    block->max_entry_speed = vmax_junction;
    
    // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
    v_allowable = max_allowable_speed(-config.acceleration,MINIMUM_PLANNER_SPEED,block->millimeters);
    block->entry_speed = min(vmax_junction, v_allowable);

    //***
    block->entry_speed = MINIMUM_PLANNER_SPEED;

    // Initialize planner efficiency flags
    // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
    // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
    // the current block and next block junction speeds are guaranteed to always be at their maximum
    // junction speeds in deceleration and acceleration, respectively. This is due to how the current
    // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
    // the reverse and forward planners, the corresponding block junction speed will always be at the
    // the maximum junction speed and may always be ignored for any speed reduction checks.
    if (block->nominal_speed <= v_allowable) 
      { block->nominal_length_flag = true; }
    else 
      { block->nominal_length_flag = false; }

    block->recalculate_flag = true; // Always calculate trapezoid for new block
  
    // Update previous path unit_vector and nominal speed
    memcpy(previous_unit_vec, unit_vec, sizeof(unit_vec)); // previous_unit_vec[] = unit_vec[]
    previous_nominal_speed = block->nominal_speed;

  } else {
    // Acceleration planner disabled. Set minimum that is required.
  //  block->entry_speed = block->nominal_speed;
    
    block->initial_rate = block->nominal_rate;
    block->final_rate = block->nominal_rate;
    block->accelerate_until = 0;
    block->decelerate_after = block->step_event_count;
    block->rate_delta = 0;
  }
  
  if (pAction->ActionType == AT_MOVE)
    block->check_endstops = false;
  else
  {
    previous_nominal_speed = 0.0;
    block->check_endstops = true;
  }
  pAction->ActionType = AT_MOVE;
  
  // Move buffer tail
  block_buffer_tail = next_buffer_tail;     
  // Update position
  memcpy(position, target, sizeof(target)); // position[] = target[]

  startpoint = pAction->target;
  
  if (acceleration_manager_enabled) { planner_recalculate(); }  
  st_wake_up();
}

void plan_buffer_wait (tActionRequest *pAction)
{
  block_t *block;
  
  // ----
  // Calculate the new buffer tail after we push this block
  uint8_t next_buffer_tail = next_block_index( block_buffer_tail );	
  
  // If the buffer is full: good! That means we are well ahead of the robot. 
  // Rest here until there is room in the buffer.
  while(block_buffer_head == next_buffer_tail) { sleep_mode(); }
  
  // Prepare to set up new block
  block = &block_buffer[block_buffer_tail];
  // ----
  
  //TODO
  
  block->action_type = pAction->ActionType;
  // every 50ms
  block->millimeters = 10;
  block->nominal_speed = 600;
  block->nominal_rate = 20*60;
  
  block->step_event_count = 1000;
  
    // Acceleration planner disabled. Set minimum that is required.
    block->entry_speed = block->nominal_speed;
    
    block->initial_rate = block->nominal_rate;
    block->final_rate = block->nominal_rate;
    block->accelerate_until = 0;
    block->decelerate_after = block->step_event_count;
    block->rate_delta = 0;
    
  block->wait_param = pAction->wait_param;

  previous_nominal_speed = 0.0;

  // Move buffer tail
  block_buffer_tail = next_buffer_tail;     

  // need to call recalc - should not affect path??
  if (acceleration_manager_enabled) { planner_recalculate(); }  
  st_wake_up();
    
}

void plan_buffer_action(tActionRequest *pAction)
{
  //TODO: check queue, wait

  switch (pAction->ActionType)
  {
  case AT_MOVE:
  case AT_MOVE_ENDSTOP:
    //plan_buffer_line (pAction->x, pAction->y, pAction->z, pAction->feed_rate, pAction->invert_feed_rate);
    plan_buffer_line (pAction);
    break;
    
  case AT_WAIT_TIME: 
    //TODO
  case AT_WAIT_TEMPERATURES:
    plan_buffer_wait (pAction);
    break;
  }
}

// Reset the planner position vector and planner speed
void plan_set_current_position_xyz(double x, double y, double z)
{
  tTarget new_pos = startpoint;
  new_pos.x = x;
  new_pos.y = y;
  new_pos.z = z;
  plan_set_current_position (&new_pos);
}

void plan_set_current_position(tTarget *new_position) 
{
  startpoint = *new_position;
  position[X_AXIS] = lround(new_position->x*(double)config.axis[X_AXIS].steps_per_mm);
  position[Y_AXIS] = lround(new_position->y*(double)config.axis[Y_AXIS].steps_per_mm);
  position[Z_AXIS] = lround(new_position->z*(double)config.axis[Z_AXIS].steps_per_mm);    
  position[E_AXIS] = lround(new_position->e*(double)config.axis[E_AXIS].steps_per_mm);    
  
  previous_nominal_speed = 0.0; // Resets planner junction speeds. Assumes start from rest.
  clear_vector_double(previous_unit_vec);
}

void plan_set_feed_rate (tTarget *new_position) 
{
  startpoint.feed_rate = new_position->feed_rate;
  startpoint.invert_feed_rate = new_position->invert_feed_rate;
}

uint8_t plan_queue_full (void)
{
  int next_buffer_tail = next_block_index( block_buffer_tail );	
  
  if (block_buffer_head == next_buffer_tail)
    return 1;
  else
    return 0;
}

uint8_t plan_queue_empty(void) 
{
  if (block_buffer_head == block_buffer_tail)
    return 1;
  else
    return 0;
}

uint8_t plan_queue_size(void) 
{

  if (block_buffer_tail >= block_buffer_head)
    return block_buffer_tail - block_buffer_head;
  else
    return block_buffer_tail - block_buffer_head + BLOCK_BUFFER_SIZE;
}

uint8_t plan_num_free_slots(void)
{
  return BLOCK_BUFFER_SIZE -1 - plan_queue_size();
}