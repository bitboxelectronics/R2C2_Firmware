#include	"movemath.h"

#include	<math.h>

float distance(float *deltas, int num_axes) {
	float sum = 0.0f;
	int i;
	for (i = 0; i < num_axes; i++) {
		sum = hypotf(sum, deltas[i]);
	}
	return sum;
}

void movemath(MATHBLOCK *mb) {
	// steps = distance * steps_per_mm
	mb->steps = mb->delta * mb->steps_per_mm;
	// time = distance / speed
	mb->move_time = mb->delta / mb->max_speed;
	// accel time = delta speed / accel
	mb->accel_time = (mb->max_speed - mb->startstop_speed) / mb->accel;
	if (mb->accel_time < 0.0)
		mb->accel_time = 0.0;
	mb->decel_time = mb->accel_time;

}
