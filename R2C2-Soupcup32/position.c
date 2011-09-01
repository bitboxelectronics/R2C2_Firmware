#include	"position.h"

#include	<type.h>
#include	<math.h>

#include	"config.h"

#define	OFFSET_STACK_DEPTH 4

POS offset_stack[NUM_AXES][OFFSET_STACK_DEPTH];
int stack_depth;

POS offset[NUM_AXES];

void get_position_absolute(int axis, POS *position) {
	update_position(axis);
	position->nm = axes[axis].position.nm;
	position->st = axes[axis].position.st;
	position->mm = axes[axis].position.mm;
}

void get_position(int axis, POS *position) {
	get_position_absolute(axis, position);
	position->nm += offset[axis].nm;
	position->st += offset[axis].st;
	position->mm += offset[axis].mm;
}

void set_position_absolute(int axis, POS *position) {
	axes[axis].position.nm = position->nm;
	axes[axis].position.st = position->st;
	axes[axis].position.mm = position->mm;
}

void set_position(int axis, POS *position) {
	POS newpos;
	newpos.nm = position->nm - offset[axis].nm;
	newpos.st = position->st - offset[axis].st;
	newpos.mm = position->mm - offset[axis].mm;
	set_position_absolute(axis, &newpos);
}

void push_offset() {
	if (stack_depth < OFFSET_STACK_DEPTH - 1) {
		int i;
		stack_depth++;
		for (i = 0; i < NUM_AXES; i++) {
			offset_stack[i][stack_depth] = offset[i];
		}
	}
}

void pop_offset() {
	if (stack_depth > 0) {
		int i;
		stack_depth--;
		for (i = 0; i < NUM_AXES; i++) {
			offset[i] = offset_stack[i][stack_depth];
		}
	}
}

int pos_to_delta_steps(int axis, POS *p) {
	return p->st - axes[axis].position.st;
}

void update_position(int axis) {
	st_to_pos(axis, axes[axis].position.st, &axes[axis].position);
}

void mm_to_pos(int axis, float mm, POS *pos) {
	pos->nm = mm * 1000000.0f;
	pos->st = ceil(mm * ((float) axes[axis].steps_per_mm));
}

void st_to_pos(int axis, int st, POS *pos) {
	pos->nm = ceil(((float) st) * 1000000.0f / axes[axis].steps_per_mm);
	pos->mm = pos->nm / 1000000.0f;
}

void nm_to_pos(int axis, int nm, POS *pos) {
	pos->mm = ((float) nm) / 1000000.0f;
	pos->st = ceil(nm * ((float) axes[axis].steps_per_mm) / 1000000.0f);
}

void step(int axis, int direction) {
	if (direction > 0)
		axes[axis].position.st++;
	else
		axes[axis].position.st--;
}
