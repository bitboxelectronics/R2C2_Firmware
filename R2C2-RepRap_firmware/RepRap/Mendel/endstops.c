#include <stdbool.h>

#include "config.h"
#include "pinout.h"


static bool test_min_stop (tPinDef limit_pin)
{
  if (read_pin (limit_pin) ^ limit_pin.active_low)
    return true;
  else
    return false;
}

// dir : 0 = towards positive, 1 = towards negative
bool hit_home_stop (unsigned axis, unsigned dir)
{
  if (config.axis[axis].home_direction < 0)
  {
    // if home direction is negative then 
    // return true if current motion is in negative direction AND min limit reached
    return test_min_stop(config.axis[axis].pin_min_limit) && (dir != 0);
  }
  else
  {
    // if home direction is positive then 
    // return true if current motion is in positive direction AND limit reached
    return test_min_stop(config.axis[axis].pin_min_limit) && (dir == 0);
  }
}




