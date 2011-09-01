#ifndef	_IO_H
#define	_IO_H

#ifndef	_LPCUSB_TYPE_H_
#ifndef	_TYPE_H_
typedef unsigned char           U8;     /**< unsigned 8-bit */
typedef unsigned short int      U16;   /**< unsigned 16-bit */
typedef unsigned int            U32;   /**< unsigned 32-bit */
#endif
#endif

typedef U16 PIN;

#define	PIN_NONE	0xFFFF
#define	PIN_INVERT	0x100


#define P0_0	0x0
#define P0_1	0x1
#define P0_2	0x2
#define P0_3	0x3
#define P0_4	0x4
#define P0_5	0x5
#define P0_6	0x6
#define P0_7	0x7
#define P0_8	0x8
#define P0_9	0x9
#define P0_10	0xa
#define P0_11	0xb
#define P0_15	0xf
#define P0_16	0x10
#define P0_17	0x11
#define P0_18	0x12
#define P0_19	0x13
#define P0_20	0x14
#define P0_21	0x15
#define P0_22	0x16
#define P0_23	0x17
#define P0_24	0x18
#define P0_25	0x19
#define P0_26	0x1a
#define P0_27	0x1b
#define P0_28	0x1c
#define P0_29	0x1d
#define P0_30	0x1e

#define P1_0	0x20
#define P1_1	0x21
#define P1_4	0x24
#define P1_8	0x28
#define P1_9	0x29
#define P1_10	0x2a
#define P1_14	0x2e
#define P1_15	0x2f
#define P1_16	0x30
#define P1_17	0x31
#define P1_18	0x32
#define P1_19	0x33
#define P1_20	0x34
#define P1_21	0x35
#define P1_22	0x36
#define P1_23	0x37
#define P1_24	0x38
#define P1_25	0x39
#define P1_26	0x3a
#define P1_27	0x3b
#define P1_28	0x3c
#define P1_29	0x3d
#define P1_30	0x3e
#define P1_31	0x3f

#define P2_0	0x40
#define P2_1	0x41
#define P2_2	0x42
#define P2_3	0x43
#define P2_4	0x44
#define P2_5	0x45
#define P2_6	0x46
#define P2_7	0x47
#define P2_8	0x48
#define P2_9	0x49
#define P2_10	0x4a
#define P2_11	0x4b
#define P2_12	0x4c
#define P2_13	0x4d

#define P3_25	0x79
#define P3_26	0x7a

#define P4_28	0x9c
#define P4_29	0x9d

#define port_from_pin(p) ((p >> 5) & 0x1F)

#define bitvalue_from_pin(p) (1 << (p & 0x1F))


int is_valid(PIN);

void set_input(PIN);
void set_output(PIN);
int read(PIN);
void write(PIN, U8);

#endif	/* _IO_H */
