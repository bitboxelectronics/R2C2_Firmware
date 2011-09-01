#include	"io.h"

#include	"lpc17xx_gpio.h"

#define	DIR_INPUT 0
#define	DIR_OUTPUT 1

static const U32 valid_mask[5] = {
	// port 0, pins 12,13,14,31 not available
	0x7FFF8FFF,
	// port 1, pins 2,3,5,6,7,11,12,13 not available
	0xFFFFC713,
	// port 2, pins 14 - 31 not available
	0x00003FFF,
	// port 3, pins 0-24, 27-31 not available
	0x06000000,
	// port 4, pins 0-27, 30,31 not available
	0x30000000
};

int is_valid(PIN p) {
	if (p > 0x9F)
		return 0;
	return valid_mask[port_from_pin(p)] & bitvalue_from_pin(p);
}

void set_input(PIN p) {
	if (is_valid(p))
		FIO_SetDir(port_from_pin(p), bitvalue_from_pin(p), DIR_INPUT);
}

void set_output(PIN p) {
	if (is_valid(p))
		FIO_SetDir(port_from_pin(p), bitvalue_from_pin(p), DIR_OUTPUT);
}

int read(PIN p) {
	if (is_valid(p))
		return (FIO_ReadValue(port_from_pin(p)) & bitvalue_from_pin(p))?-1:0;
	return 0;
}

void write(PIN p, U8 value) {
	if (is_valid(p) == 0)
		return;
	if (p & PIN_INVERT)
		value = (value)?0:1;
	if (value)
		FIO_SetValue(port_from_pin(p), bitvalue_from_pin(p));
	else
		FIO_ClearValue(port_from_pin(p), bitvalue_from_pin(p));
}

void set_pin(PIN p) {
	FIO_SetValue(port_from_pin(p), bitvalue_from_pin(p));
}

void clear_pin(PIN p) {
	FIO_ClearValue(port_from_pin(p), bitvalue_from_pin(p));
}
