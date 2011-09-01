#ifndef	_CONFIG_H
#define	_CONFIG_H

#include	"io.h"

#include	"position.h"

typedef float UNIT;

enum {
	X,
	Y,
	Z,
	E,
	NUM_AXES
} axis_names;

enum {
	extruder,
	bed,
	NUM_HEATERS
} heater_names;

typedef struct {
	UNIT	steps_per_mm;
	UNIT	accel;
	UNIT	length;
	POS		position;
	PIN		step;
	PIN		dir;
	PIN		enable;
	PIN		min;
	PIN		max;
} axis_config;

typedef struct {
	PIN sensor;
	PIN heater;
	struct {
		U16 adc_reading;
		U16 temperature;
	} temptable[32];
} sensor_heater_pair;

extern axis_config axes[NUM_AXES];
extern sensor_heater_pair heaters[NUM_HEATERS];

void pins_init(void);

#define	AXIS_DISABLED 1
#define	AXIS_ENABLED  0

#endif	/* _CONFIG_H */
