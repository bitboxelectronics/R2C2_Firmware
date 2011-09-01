#ifndef	_GCODE_PARSE_H
#define	_GCODE_PARSE_H

#include	"position.h"

#define	STATE_RELATIVE	1

extern unsigned int state_flags;

POS target[NUM_AXES];

unsigned char seen(unsigned char c);

#endif	/* _GCODE_PARSE_H */
