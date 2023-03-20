#include "s15.h"

/* Bit 16: Always 0
   Bit 15: Sign bit */

sint15 s15(sint16 i)
{
    // If negative, subtract 1 so we get one's complement
    if (i < 0) i--;

    // Copy the sign bit to bit 15
    // This truncates the number to 15 bits and preserves the sign
    i = ((i & 0x8000) >> 1) | (i & 0x3FFF);

    return (sint15)i;
}

sint15 s15_u(uint16 i)
{
    return (sint15)(i & 0x7FFF);
}

sint16 s16(sint15 i)
{
    // Add 1 to negative numbers to get two's complement
    // -0 will correctly be 0 afterwards (with overflow)
    if (i & 0x4000) i++;

    // Copy bit 15 to bit 16 to convert sign and sign extend the additional bit
    i = ((i & 0x4000) << 1) | (i & 0x7FFF);

    return i;
}

sint15 s15_add(sint15 a, sint15 b)
{
    // Do normal addition, since this is done correctly
    sint16 i = a + b;

    // Value will be negative when overflown
    if (i < 0)
    {
        // Remove sign bit and add overflow bit
        i &= ~0x8000;

        i++;
    }

    return (sint15)i;
}

sint15 s15_sub(sint15 a, sint15 b)
{
    // Add with ones complement
    return s15_add(a, (~b & 0x7FFF));
}

sint15 s15_abs(sint15 a)
{
    // If negative, invert
    if (a & 0x4000) a = ~a;

    return a;
}

sint15 s15_ccs(sint15 a)
{
    a = s15_abs(a);
    if (a > 0) a--;

    return a;
}