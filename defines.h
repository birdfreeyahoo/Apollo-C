#ifndef DEFINES_H
#define DEFINES_H

typedef unsigned short uint16;
typedef signed short sint16;
typedef sint16 sint15;
typedef unsigned char uint8;
typedef unsigned char bool;

#define TRUE 1
#define FALSE 0

// 1 - indexed bit masks
#define BITMASK1(bitno) (1 << (bitno - 1))

#define SETLOC(x) 

// Memory defines
#define ISR_START 04000

#endif