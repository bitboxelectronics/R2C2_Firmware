#include	"position.h"

#include	<type.h>
#include	<math.h>

#include	"config.h"

inline int pos_to_delta_steps(int axis, float p_mm) {
	return ceil(p_mm * axes[axis].steps_per_mm) - axes[axis].st;
}

void steps_to_position(int axis) {
	axes[axis].nm = ceil(((float) axes[axis].st) * 1000000.0f / axes[axis].steps_per_mm);
	axes[axis].mm = axes[axis].nm / 1000000.0f;
}

void step(int axis, int direction) {
	if (direction > 0)
		axes[axis].st++;
	else
		axes[axis].st--;
}