#ifndef S15_H
#define S15_H

#include "defines.h"

#define S15_MINUS_ZERO 0xFFFF
#define S15_MAXPOS 0x3FFF
#define S15_MAXNEG ~S15_MAXPOS
#define S15_SIGN   0x4000

int15_t s15(int16_t i);
int15_t s15_u(uint16_t i);

int16_t s16(int15_t i);


/* Only use those after conversion! */
int15_t s15_add(int15_t a, int15_t b);
int15_t s15_sub(int15_t a, int15_t b);

int15_t s15_abs(int15_t a);

/* abs(x) - 1, but at least 0 */
int15_t s15_ccs(int15_t a);

void s15_mulDP(int15_t a, int15_t b, int15_t *result);

/* Corrects overflow so wraps to 0 without sign change */
int15_t s15_ovfC(int15_t a);

/* Checks for overflow and returns 1 for + ovf and -1 for -ovf and 0 for no ovf*/
int15_t s15_getOvf(int15_t a);

/* Sign extends to bit 16*/
int15_t s15_se(int15_t a);

#endif