#include	 "acceleration.h"

/**

	From http://www.eetimes.com/design/embedded/4006438/Generate-stepper-motor-speed-profiles-in-real-time

	f = timer frequency (equal to clock frequency on atmega, 16MHz)
	c = time between steps
	w = speed
	a = step angle or step distance, we use 1/steps_per_mm
	w' = acceleration
	n = ramp counter (integer)
	i = step number (integer)

	[Equation 7] *** IMPORTANT- THIS IS HOW WE SET THE ACCELERATION AT THE START OF RAMPING ***
		c[0] = f.sqrt(2.a/w')

	[Equation 13] *** IMPORTANT- WE CALCULATE THIS EVERY STEP ***
		c[i] = c[i-1] - [ 2.c[i-1] / (4.n[i] + 1) ]

	[Equation 14]
		c[i] = c[i-1] - [ 2.c[i-1] / (4.n[i - m] + 1) ] , i < m

	[Equation 15]
		c[0] = 0.676.f.sqrt(2.a / w')

	[Equation 16] *** IMPORTANT- THIS IS HOW WE START (OR END) A RAMP AT A SPECIFIC SPEED ***
		n = [ w^2 / 2.a.w' ]
*/

#include	<stdlib.h>
#include	<math.h>

#include	"gcode_parse.h"
#include	"machine.h"
#include	"move.h"
#include	"serial.h"

#define amask(c)	((axis_mask & AXIS_ ## c)?255:0)

void movemath() {
	float xs = 0.0, ys = 0.0, zs = 0.0, es = 0.0;
	float da, dd;

	uint8_t axis_mask = 0;

	int32_t x_steps = 0, y_steps = 0, z_steps = 0, e_steps = 0;
	float xd = 0.0, yd = 0.0, zd = 0.0, ed = 0.0;

// 	printf_P(PSTR("doing math- distance..."));

	if (seen('X')) {
		if (state_flags & STATE_RELATIVE)
			x_steps = labs(lround(words[idx('X')].f * x_steps_per_mm));
		else
			x_steps = labs(lround(words[idx('X')].f * x_steps_per_mm) - s_endpoint.X);
// 		s_endpoint.X += x_steps;
		xd = ((float) x_steps) / x_steps_per_mm;
		if (x_steps != 0)
			axis_mask |= AXIS_X;
	}
	if (seen('Y')) {
		if (state_flags & STATE_RELATIVE)
			y_steps = labs(lround(words[idx('Y')].f * y_steps_per_mm));
		else
			y_steps = labs(lround(words[idx('Y')].f * y_steps_per_mm) - s_endpoint.Y);
// 		s_endpoint.Y += y_steps;
		yd = ((float) y_steps) / y_steps_per_mm;
		if (y_steps != 0)
			axis_mask |= AXIS_Y;
	}
	if (seen('Z')) {
		if (state_flags & STATE_RELATIVE)
			z_steps = labs(lround(words[idx('Z')].f * z_steps_per_mm));
		else
			z_steps = labs(lround(words[idx('Z')].f * z_steps_per_mm) - s_endpoint.Z);
// 		s_endpoint.Z += z_steps;
		zd = ((float) z_steps) / z_steps_per_mm;
		if (z_steps != 0)
			axis_mask |= AXIS_Z;
	}
	if (seen('E')) {
		if (state_flags & STATE_RELATIVE)
			e_steps = labs(lround(words[idx('E')].f * e_steps_per_mm));
		else
			e_steps = labs(lround(words[idx('E')].f * e_steps_per_mm) - s_endpoint.E);
// 		s_endpoint.E += e_steps;
		ed = ((float) e_steps) / e_steps_per_mm;
		if (e_steps != 0)
			axis_mask |= AXIS_E;
	}

	if (axis_mask == 0)
		return;

	// work out distance
	if (seen(DISTANCE) == 0) {
		float d;
		if (amask(X) && amask(Y) && amask(Z) == 0)
			d = hypot(xd, yd);
		else if (amask(X) == 0 && amask(Y) == 0 && amask(Z))
			d = zd;
		else if (amask(X) == 0 && amask(Y) == 0 && amask(Z) == 0 && amask(E))
			d = ed;
		else {
			d = hypot(xd, yd);
			d = hypot(d, zd);
		}
		set_f(DISTANCE, d);
	}

// 	printf_P(PSTR("(%f) OK, time..."), words[idx(DISTANCE)].f);

	// work out time
	// time = distance / speed
	if (seen(TIME) == 0) {
		float f = words[idx('F')].f / 60.0;
		if (f < 0.1)
			f = 0.1;
		set_f(TIME, words[idx(DISTANCE)].f / f);
	}

// 	printf_P(PSTR("(%f) OK, accel..."), words[idx(TIME)].f);

	if (amask(X)) {
		if (seen(XS) == 0)
			set_f(XS, xs = (xd / words[idx(TIME)].f));
		else
			xs = words[idx(XS)].f;
	}

	if (amask(Y)) {
		if (seen(YS) == 0)
			set_f(YS, ys = (yd / words[idx(TIME)].f));
		else
			ys = words[idx(YS)].f;
	}

	if (amask(Z)) {
		if (seen(ZS) == 0)
			set_f(ZS, zs = (zd / words[idx(TIME)].f));
		else
			zs = words[idx(ZS)].f;
	}

	if (amask(E)) {
		if (seen(ES) == 0)
			set_f(ES, es = (ed / words[idx(TIME)].f));
		else
			es = words[idx(ES)].f;
	}

// 	printf_P(PSTR("[speeds: %g,%g,%g,%g]"), xs, ys, zs, es);

	// work out accel profiles
	if (seen(ACCEL_DISTANCE) == 0) {
		// distance to accel to speed w at acceleration w' = w**2 / 2w'
		da = 0.0;

		if (amask(X))
			da = fmax(da, square(xs) / 2.0 / x_accel);
		if (amask(Y))
			da = fmax(da, square(ys) / 2.0 / y_accel);
		if (amask(Z))
			da = fmax(da, square(zs) / 2.0 / z_accel);
		if (amask(E))
			da = fmax(da, square(es) / 2.0 / e_accel);

		set_f(ACCEL_DISTANCE, da);
	}
	else
		da = words[idx(ACCEL_DISTANCE)].f;

	// now apply this accel ramp to other axes
	// d = s**2/2a, therefore a = s**2/2d

	// 				printf_P(PSTR("[Accel: %g,%g,%g,%g]"), xa, ya, za, ea);

	if (seen(DECEL_DISTANCE) == 0) {
		// distance to decel from speed w at deceleration w' = w**2 / 2 / w'
		dd = 0.0;
		if (amask(X))
			dd = fmax(dd, square(xs) / 2.0 / x_decel);
		if (amask(Y))
			dd = fmax(dd, square(ys) / 2.0 / y_decel);
		if (amask(Z))
			dd = fmax(dd, square(zs) / 2.0 / z_decel);
		if (amask(E))
			dd = fmax(dd, square(es) / 2.0 / e_decel);
		
		// we start decelerating before reaching top speed, recalculate dd
		if (da + dd >= words[idx(DISTANCE)].f) {
			float ddn = da + dd;
			dd = 0.0;
			if (amask(X))
				dd = fmax(dd, ddn * x_decel / (x_accel + x_decel));
			if (amask(Y))
				dd = fmax(dd, ddn * y_decel / (y_accel + y_decel));
			if (amask(Z))
				dd = fmax(dd, ddn * z_decel / (z_accel + z_decel));
			if (amask(E))
				dd = fmax(dd, ddn * e_decel / (e_accel + e_decel));
		}
		
		set_f(DECEL_DISTANCE, dd);
	}
	else
		dd = words[idx(DECEL_DISTANCE)].f;
	
	if (amask(X)) {
		if (seen(XMC) == 0)
			set_i(XMC, lround(((float) F_CPU) / (xs * x_steps_per_mm)) * 256.0);
		if (seen(XC0) == 0)
			set_i(XC0, lround(((float) F_CPU) * sqrt(4.0 * da / square(xs) / x_steps_per_mm) * 256.0));
		if (seen(XDS) == 0)
			set_i(XDS, lround(dd * words[idx(DISTANCE)].f / xd * x_steps_per_mm));
	}

	if (amask(Y)) {
		if (seen(YMC) == 0)
			set_i(YMC, lround(((float) F_CPU) / (ys * y_steps_per_mm)) * 256.0);
		if (seen(YC0) == 0)
			set_i(YC0, lround(((float) F_CPU) * sqrt(4.0 * da / square(ys) / y_steps_per_mm) * 256.0));
		if (seen(YDS) == 0)
			set_i(YDS, lround(square(ys) * y_steps_per_mm / 2.0 / dd));
	}

	if (amask(Z)) {
		if (seen(ZMC) == 0)
			set_i(ZMC, lround(((float) F_CPU) / (zs * z_steps_per_mm)) * 256.0);
		if (seen(ZC0) == 0)
			set_i(ZC0, lround(((float) F_CPU) * sqrt(4.0 * da / square(zs) / z_steps_per_mm) * 256.0));
		if (seen(ZDS) == 0)
			set_i(ZDS, lround(square(zs) * z_steps_per_mm / 2.0 / dd));
	}

	if (amask(E)) {
		if (seen(EMC) == 0)
			set_i(EMC, lround(((float) F_CPU) / (es * e_steps_per_mm)) * 256.0);
		if (seen(EC0) == 0)
			set_i(EC0, lround(((float) F_CPU) * sqrt(4.0 * da / square(es) / e_steps_per_mm) * 256.0));
		if (seen(EDS) == 0)
			set_i(EDS, lround(square(es) * e_steps_per_mm / 2.0 / dd));
	}

	// 	printf_P(PSTR("(%lu,%lu,%lu,%lu/%lu,%lu,%lu,%lu) OK, decel..."), words[idx(XC0)].u, words[idx(YC0)].u, words[idx(ZC0)].u, words[idx(EC0)].u, words[idx(XMC)].u, words[idx(YMC)].u, words[idx(ZMC)].u, words[idx(EMC)].u);

// 	printf_P(PSTR(" OK\n"));
}
