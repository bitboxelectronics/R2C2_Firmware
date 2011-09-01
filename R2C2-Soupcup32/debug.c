#include	"debug.h"

#include	"config.h"

int num = 0;

void debug_next() {
	debug_num(++num);
}

void debug_num(int n) {
	write(axes[X].step, n & 1);
	write(axes[Y].step, n & 2);
	write(axes[Z].step, n & 4);
	write(axes[E].step, n & 8);
	num = n;
}
