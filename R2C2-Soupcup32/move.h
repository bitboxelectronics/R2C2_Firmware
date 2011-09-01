#ifndef	_MOVE_H
#define	_MOVE_H

#include	<stdint.h>

#define		MOVEBUFFER_SIZE	8

#define		DEFINE_TEMP_SENSOR(...)
#define		DEFINE_HEATER(...)

#include	"config.h"

typedef struct {
	uint32_t	directions;
	uint32_t	steps[NUM_AXES];
	uint32_t	c;
	uint32_t	ds;
	uint32_t	minc;
	uint32_t	rs;
} move;

extern volatile uint8_t mb_head;
extern volatile uint8_t mb_tail;

extern volatile move movebuffer[MOVEBUFFER_SIZE];

#define	 fracmult(m, f) _fracmult(m, ((uint32_t) (f * 0x80000000)))
uint32_t _fracmult(uint32_t multiplicand, uint32_t fraction);

uint8_t	queue_full(void);
uint8_t queue_empty(void);
#define	queue_wait()	do { loopstuff(); } until (queue_empty() && movebuffer[mb_tail].live == 0)
#define	queue_wait_1()	do { loopstuff(); } while (queue_full())

void enqueue(void);

void move_start(void);
void move_step(void);

#endif	/* _MOVE_H */
