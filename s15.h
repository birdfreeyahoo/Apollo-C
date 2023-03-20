#ifndef S15_H
#define S15_H

#include "defines.h"

#define S15_MINUS_ZERO 0x7FFF

sint15 s15(sint16 i);
sint15 s15_u(uint16 i);

sint16 s16(sint15 i);


/* Only use those after conversion! */
sint15 s15_add(sint15 a, sint15 b);
sint15 s15_sub(sint15 a, sint15 b);

sint15 s15_abs(sint15 a);

/* abs(x) - 1, but at least 0 */
sint15 s15_ccs(sint15 a);

#endif