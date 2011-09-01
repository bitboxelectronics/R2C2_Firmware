#ifndef	_POSITION_H
#define	_POSITION_H

typedef struct {
	int nm;
	int st;
	float mm;
} POS;

void get_position_absolute(int axis, POS *position);
void get_position(int axis, POS *position);

void set_position_absolute(int axis, POS *position);
void set_position(int axis, POS *position);



void steps_to_position(int axis);

void set_offset(int axis, POS *offset);

// offset stack
void push_offset(void);
void pop_offset(void);

int pos_to_delta_steps(int axis, POS *p);

void update_position(int axis);

void mm_to_pos(int axis, float mm, POS *pos);
void st_to_pos(int axis, int st, POS *pos);
void nm_to_pos(int axis, int nm, POS *pos);

#endif	/* _POSITION_H */
