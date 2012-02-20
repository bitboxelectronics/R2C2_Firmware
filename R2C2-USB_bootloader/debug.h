#ifndef	_DEBUG_H
#define	_DEBUG_H

#ifdef	UARTDEBUG
#include	<stdint.h>

// functions for sending hexadecimal
void serwrite_hex4(uint8_t v);
void serwrite_hex8(uint8_t v);
void serwrite_hex16(uint16_t v);
void serwrite_hex32(uint32_t v);

// functions for sending decimal
#define	serwrite_uint8(v)		serwrite_uint32(v)
#define	serwrite_int8(v)		serwrite_int32(v)
#define	serwrite_uint16(v)	serwrite_uint32(v)
#define	serwrite_int16(v)		serwrite_int32(v)

void serwrite_uint32(uint32_t v);
void serwrite_int32(int32_t v);

void serwrite_uint32_vf(uint32_t v, uint8_t fp);
void serwrite_int32_vf(int32_t v, uint8_t fp);

void sersendf(char *format, ...)		__attribute__ ((format (printf, 1, 2)));

#define DBG(format, ...) sersendf(format "\n", ##__VA_ARGS__)

#else	/* ifdef DEBUG */

	#define	DBG(...) do {} while(0)

#endif	/* ifdef UARTDEBUG */

#endif	/* _DEBUG_H */
