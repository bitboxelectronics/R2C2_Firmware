/*
        utility functions
*/

#include <math.h>

double calc_distance( double dx, double dy )
{
  return (sqrt(dx*dx + dy*dy));
}

double calc_distance_3( double dx, double dy, double dz )
{
  return (sqrt(dx*dx + dy*dy + dz*dz));
}

