#ifndef FLAGS_H
#define FLAGS_H

#include "defines.h"

#define MAKE_FLAG(flag, bit) (flag * 15 + (bit - 1))

typedef enum {
    FLAG_DSKY = MAKE_FLAG(5, 15)
} Flags_t;

#define FLAG_SET(flag) (Flags[flag / 15] |= (1 << (flag % 15)))
#define FLAG_GET(flag) (Flags[flag / 15] & (1 << (flag % 15)))

extern int16_t Flags[6];

#endif