#include	"debug.h"

#ifdef	UARTDEBUG

#include	<stdarg.h>
#include	<math.h>

#include	"uart.h"

/** write a single hex digit
	\param v hex digit to write, higher nibble ignored
*/
void serwrite_hex4(uint8_t v) {
	v &= 0xF;
	if (v < 10)
		serial_writechar('0' + v);
	else
		serial_writechar('A' - 10 + v);
}

/** write a pair of hex digits
	\param v byte to write. One byte gives two hex digits
*/
void serwrite_hex8(uint8_t v) {
	serwrite_hex4(v >> 4);
	serwrite_hex4(v & 0x0F);
}

/** write four hex digits
	\param v word to write
*/
void serwrite_hex16(uint16_t v) {
	serwrite_hex8(v >> 8);
	serwrite_hex8(v & 0xFF);
}

/** write eight hex digits
	\param v long word to write
*/
void serwrite_hex32(uint32_t v) {
	serwrite_hex16(v >> 16);
	serwrite_hex16(v & 0xFFFF);
}

/// list of powers of ten, used for dividing down decimal numbers for sending, and also for our crude floating point algorithm
const uint32_t powers[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

/** write decimal digits from a long unsigned int
	\param v number to send
*/
void serwrite_uint32(uint32_t v) {
	uint8_t e, t;

	for (e = 9; e > 0; e--) {
		if (v >= powers[e])
			break;
	}

	do
	{
		for (t = 0; v >= powers[e]; v -= powers[e], t++);
		serial_writechar(t + '0');
	}
	while (e--);
}

/** write decimal digits from a long signed int
	\param v number to send
*/
void serwrite_int32(int32_t v) {
	if (v < 0) {
		serial_writechar('-');
		v = -v;
	}

	serwrite_uint32(v);
}

/** write decimal digits from a long unsigned int
\param v number to send
*/
void serwrite_uint32_vf(uint32_t v, uint8_t fp) {
	uint8_t e, t;

	for (e = 9; e > 0; e--) {
		if (v >= powers[e])
			break;
	}

	if (e < fp)
		e = fp;

	do
	{
		for (t = 0; v >= powers[e]; v -= powers[e], t++);
		serial_writechar(t + '0');
		if (e == fp)
			serial_writechar('.');
	}
	while (e--);
}

/** write decimal digits from a long signed int
\param v number to send
*/
void serwrite_int32_vf(int32_t v, uint8_t fp) {
	if (v < 0) {
		serial_writechar('-');
		v = -v;
	}

	serwrite_uint32_vf(v, fp);
}

// void serwrite_double(double f) {
// 	int precision = 6;
// 	serwrite_int32(floor(f));
// 	f -= floor(f);
// 	if (f != 0) {
// 		serial_writechar('.');
// 		while ((f != 0) && (precision-- != 0)) {
// 			f *= 10;
// 			serial_writechar('0' + floor(f));
// 			f -= floor(f);
// 		}
// 	}
// }

void sersendf(char * format, ...) {
	va_list args;
	va_start(args, format);

	uint16_t i = 0;
	uint8_t c = 1, j = 0;
	while ((c = format[i++])) {
		if (j) {
			switch(c) {
				case 's':
					j = 1;
					break;
				case 'l':
					j = 4;
					break;
				case 'u':
					if (j == 4)
						serwrite_uint32(va_arg(args, uint32_t));
					else
						serwrite_uint16(va_arg(args, int));
					j = 0;
					break;
				case 'd':
					if (j == 4)
						serwrite_int32(va_arg(args, int32_t));
					else
						serwrite_int16(va_arg(args, int));
					j = 0;
					break;
				case 'c':
					serial_writechar(va_arg(args, int));
					j = 0;
					break;
				case 'x':
					serial_writechar('0');
					serial_writechar('x');
					if (j == 4)
						serwrite_hex32(va_arg(args, uint32_t));
					else if (j == 1)
						serwrite_hex8(va_arg(args, int));
					else
						serwrite_hex16(va_arg(args, int));
					j = 0;
					break;
				case 'p':
					serwrite_hex32(va_arg(args, uint32_t));
				case 'q':
					serwrite_int32_vf(va_arg(args, int32_t), 3);
					j = 0;
					break;
				case '%':
					serial_writechar('%');
					j = 0;
					break;
// 				case 'g':
// 					serwrite_double(va_arg(args, double));
// 					j = 0;
// 					break;
				default:
					serial_writechar(c);
					j = 0;
					break;
			}
		}
		else {
			if (c == '%') {
				j = 2;
			}
			else {
				serial_writechar(c);
			}
		}
	}
	va_end(args);
}

#endif	/* ifdef UARTDEBUG */
