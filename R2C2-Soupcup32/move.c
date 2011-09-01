#include	"move.h"

#include	<math.h>
#include	<stdlib.h>

#include	"timer.h"
#include	"gcode_parse.h"

#define		DEFINE_TEMP_SENSOR(...)
#define		DEFINE_HEATER(...)

#include	"config.h"

#define	max(a, b) (((a) >= (b))?(a):(b))

volatile uint8_t mb_head = 0;
volatile uint8_t mb_tail = 0;

volatile move movebuffer[MOVEBUFFER_SIZE] __attribute__ ((section (".bss")));

// microseconds
#define JITTER_US			2L
#define	JITTER_CYCLES	((F_CPU / 1000000L) * JITTER_US)

// move runtimes - variables that don't need to be precalculated
uint32_t dt;
uint32_t x_delta, y_delta, z_delta, e_delta;
uint8_t axis_mask;
uint32_t ts, sr;
int32_t c, n;
int32_t x_bc, y_bc, z_bc, e_bc;

uint32_t _fracmult(uint32_t multiplicand, uint32_t fraction) {
	uint32_t r = 0;
	while (fraction && multiplicand) {
		if (fraction & 0x80000000)
			r += multiplicand;
		fraction <<= 1L;
		multiplicand >>= 1L;
	}
	return r;
}

uint8_t queue_full() {
	return (((mb_tail - mb_head - 1) & (MOVEBUFFER_SIZE - 1)) == 0)?255:0;
}

uint8_t queue_empty() {
	return (mb_head == mb_tail)?255:0;
}

void enqueue() {
	uint8_t i;
	int32_t steps[NUM_AXES];

	for (i = 0; i < NUM_AXES; i++) {
		steps[i] = target[i].st - axes[i].position.st;
	}

	do {
		loopstuff();
	} while (queue_full());

	timer_register_callback(&move_step);

	move *m = (move *) &movebuffer[mb_head];

	m->flags = 0;

	ts = 0;

	for (i = 0; i < NUM_AXES; i++) {
		if (steps[i] >= 0)
			m->directions |= (1 << i)
		else
			m->directions &= ~(1 << i);
		steps[i] = labs(steps[i]);
		m->steps[i] = steps[i];
		ts = max(ts, steps[i]);
	}

	if (ts == 0)
		return;

	sr = ts;
	
	m->rs = ts;
	
	for (i = 0; i < NUM_AXES; i++) {
		if (ts == steps[i]) {
			m->ds = ds[i];
			m->c = c[i];
			m->minc = minc[i];
		}
	}

	disableTimerInterrupt();

	mb_head = (mb_head + 1) & (MOVEBUFFER_SIZE - 1);

	if (movebuffer[mb_tail].rs == 0) {
		move_start();
		enableTimerInterrupt();
	}
	else
		enableTimerInterrupt();
}

void move_start() {
	// we don't need this to be volatile in interrupt context, and it will just slow us down
	move *m = (move *) &movebuffer[mb_tail];
	uint8_t i;

	ts = 0;
	axis_mask = 0;

	for (i = 0; i < NUM_AXES; i++) {
		write(axes[i].dir, m->directions & (1 << i));
		delta[i] = m->steps[i];
		ts = max(ts, m->steps[i]);
		if (m->steps[i] != 0)
			axis_mask |= (1 << i);
		bc[i] = delta[i] >> 1;
	}

	if (ts == 0)
		return;

	n = 5L;
	c = m->c;;

	timer_set(c);
}

void move_step() {
	if (queue_empty())
		timer_set(SystemCoreClock / 1000);

	// we don't need this to be volatile in interrupt context, and it will just slow us down
	move *m = (move *) &movebuffer[mb_tail];
	uint8_t i;

	for (i = 0; i < NUM_AXES; i++) {
		if (axis_mask & (1 << i)) {
			bc[i] -= delta[i];
			if (bc[i] <= 0) {
				axis_step(i);
				if (--m->steps[i] == 0)
					axis_mask &= ~(1 << i);
				bc[i] += ts;
			}
		}
	}

	if (sr == m->ds)
		n = (sr * -4L) + 1L;

	if (n != 0) {
		if (n == 5L)
			c *= 0.4056;
		else
			c = c - ((c * 2L) / n);

		n += 4L;

		if (c < m->minc)
			n = 0;
	}

	sr--;
	if (sr == 0) {
		// end of move
		m->live = 0;

		mb_tail++;
		mb_tail &= MOVEBUFFER_SIZE - 1;

		if (queue_empty() == 0) {
			// start next move
			move_start();
		}
		else {
			// end of buffer
			timer_set(SystemCoreClock / 1000);
		}
	}
	else {
		if (m->c < m->minc)
			timer_set(m->minc);
		else
			timer_set(m->c);
	}

	unstep();
}
