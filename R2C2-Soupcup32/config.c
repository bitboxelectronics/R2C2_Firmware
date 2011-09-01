#include	"config.h"

#include	<math.h>

#include	"lpc17xx_pinsel.h"
#include	"io.h"

axis_config axes[NUM_AXES] = {
	// X
	{
		80.0f,		// steps_per_mm
		10.0f,		// accel
		200.0f,		// length
		0,				// nanometers
		0,				// steps
		0.0f,			// millimeters
		P1_20,		// step pin
		P1_23,		// dir pin
		P1_24,		// enable pin
		P2_6,			// min pin
		PIN_NONE	// max pin
	},
	// Y
	{
		80.0f,		// steps_per_mm
		10.0f,		// accel
		200.0f,		// length
		0,				// nanometers
		0,				// steps
		0.0f,			// millimeters
		P1_25,		// step pin
		P1_26,		// dir pin
		P1_28,		// enable pin
		P2_7,			// min pin
		PIN_NONE	// max pin
	},
	// Z
	{
		400.0f,		// steps_per_mm
		1.0f,			// accel
		140.0f,		// length
		0,				// nanometers
		0,				// steps
		0.0f,			// millimeters
		P1_29,		// step pin
		P0_0,			// dir pin
		P0_1,			// enable pin
		P2_8,			// min pin
		PIN_NONE	// max pin
	},
	// E
	{
		643.0f,		// steps_per_mm
		40.0f,		// accel
		INFINITY,	// length
		0,				// nanometers
		0,				// steps
		0.0f,			// millimeters
		P0_10,		// step pin
		P0_11,		// dir pin
		P2_10,		// enable pin
		PIN_NONE,	// min pin
		PIN_NONE	// max pin
	}
};

sensor_heater_pair heaters[NUM_HEATERS] = {
	// extruder
	{
		P0_2,			// sensor
		P2_4,			// heater
		{					// temp table
			{ 899, 310 },
			{ 1009, 300 },
			{ 1119, 290 },
			{ 1240, 280 },
			{ 1372, 270 },
			{ 1517, 260 },
			{ 1673, 250 },
			{ 1839, 240 },
			{ 2015, 230 },
			{ 2198, 220 },
			{ 2385, 210 },
			{ 2573, 200 },
			{ 2759, 190 },
			{ 2940, 180 },
			{ 3112, 170 },
			{ 3270, 160 },
			{ 3415, 150 },
			{ 3544, 140 },
			{ 3655, 130 },
			{ 3750, 120 },
			{ 3830, 110 },
			{ 3894, 100 },
			{ 3946, 90 },
			{ 3986, 80 },
			{ 4017, 70 },
			{ 4041, 60 },
			{ 4058, 50 },
			{ 4070, 40 },
			{ 4079, 30 },
			{ 4085, 20 },
			{ 4089, 10 },
			{ 4092, 0 },
		}
	},
	// bed
	{
		P0_3,
		P2_5,
		{				// temp table
			{ 30, 310 },
			{ 36, 300 },
			{ 42, 290 },
			{ 48, 280 },
			{ 56, 270 },
			{ 65, 260 },
			{ 76, 250 },
			{ 90, 240 },
			{ 106, 230 },
			{ 126, 220 },
			{ 151, 210 },
			{ 182, 200 },
			{ 220, 190 },
			{ 268, 180 },
			{ 328, 170 },
			{ 402, 160 },
			{ 496, 150 },
			{ 614, 140 },
			{ 761, 130 },
			{ 941, 120 },
			{ 1161, 110 },
			{ 1420, 100 },
			{ 1719, 90 },
			{ 2048, 80 },
			{ 2394, 70 },
			{ 2737, 60 },
			{ 3056, 50 },
			{ 3335, 40 },
			{ 3563, 30 },
			{ 3738, 20 },
			{ 3866, 10 },
			{ 3954, 0 },
		}
	}
};

void pins_init() {
	int i;
	PINSEL_CFG_Type p;

	for (i = 0; i < NUM_AXES; i++) {
		axis_config *axis = &axes[i];
		set_output(axis->enable); write(axis->enable, AXIS_DISABLED);
		set_output(axis->step); write(axis->step, 0);
		set_output(axis->dir); write(axis->dir, 0);
		set_input(axis->min);
		set_input(axis->max);
	}

	for (i = 0; i < NUM_HEATERS; i++) {
		sensor_heater_pair *pair = &heaters[i];
		set_output(pair->heater); write(pair->heater, 0);

		p.Portnum = port_from_pin(pair->sensor);
		p.Pinnum = pair->sensor & 0x1F;
		p.Pinmode = PINSEL_PINMODE_TRISTATE;
		p.OpenDrain = PINSEL_PINMODE_NORMAL;
		switch(pair->sensor) {
			case P0_23:	// AD0.0
			case P0_24:	// AD0.1
			case P0_25:	// AD0.2
			case P0_26:	// AD0.3
				p.Funcnum = 1;
				break;
			case P1_30:	// AD0.4
			case P1_31:	// AD0.5
				p.Funcnum = 3;
				break;
			case P0_3:	// AD0.6
			case P0_2:	// AD0.7
				p.Funcnum = 2;
				break;
		}
		PINSEL_ConfigPin(&p);
	}
}
