#include "s15.h"

/* Bit 16: Sign-extended / overflow
   Bit 15: Sign bit */

int15_t s15(int16_t i)
{
    // If negative, subtract 1 so we get one's complement
    if (i < 0) i--;

    // Copy the sign bit to bit 15
    // This truncates the number to 15 bits and preserves the sign
    // Keep bit 16 as overflow extension
    i = ((i & 0x8000) >> 1) | (i & 0xBFFF);

    return (int15_t)i;
}

int15_t s15_u(uint16_t i)
{
    // Delete bits 15 and 16
    return (int15_t)(i & 0x3FFF);
}

int16_t s16(int15_t i)
{
    // Add 1 to negative numbers to get two's complement
    // -0 will correctly be 0 afterwards (with overflow)
    if (i & 0x4000) i++;

    // Copy bit 15 to bit 16 to convert sign and sign extend the additional bit
    i = ((i & 0x4000) << 1) | (i & 0x7FFF);

    return i;
}

int15_t s15_add(int15_t a, int15_t b)
{
    // Do normal addition, since this is done correctly
    uint32_t i = (uint32_t)((uint16_t)a) + (uint32_t)((uint16_t)b);

    // Check for - to + transition, that is when bit 17 is set
    if (i & 0x10000)
    {
        // Keep also overflow bit
        i++;
        i &= 0xFFFF;
    }

    return (int15_t)i;
}

int15_t s15_sub(int15_t a, int15_t b)
{
    // Add with ones complement
    return s15_add(a, (~b & 0xFFFF));
}

int15_t s15_abs(int15_t a)
{
    // If negative, invert
    if (a & 0x4000) a = ~a;

    return a;
}

int15_t s15_ccs(int15_t a)
{
    a = s15_abs(a);
    if (a > 0) a--;

    return a;
}

// a is accumulator word (for determining -0)
void s15_mulDP(int15_t a, int15_t b, int15_t *result)
{
    // Convert to 16 bit
    int16_t a16 = s16(a);
    int16_t b16 = s16(b);

    // Check for -0
    if(((a == S15_MINUS_ZERO) && (b16 > 0)) || ((a == 0) && (b16 < 0)))
    {
        result[0] = S15_MINUS_ZERO;
        result[1] = S15_MINUS_ZERO;
        return;
    }

    // Multiply
    int32_t i = a16 * b16;

    // One complement
    if(i < 0) i--;

    // Get s15 sign
    int16_t sign = (i & 0x80000000) >> 17;
    // sign-extend overflow
    int16_t overflow = (i & 0x80000000) >> 16;

    // Low word:
    // Contains the lower 14 bits of the result and the sign bit
    result[1] = (int15_t)((int16_t)(i & 0x3FFF) | sign | overflow);
    
    // High word:
    // Contains the next 14 bits of the result and the sign bit
    result[0] = (int15_t)((int16_t)(i & 0x0FFFC000) | sign | overflow);
}

int15_t s15_ovfC(int15_t a)
{
    // Move bit 16 to 15
    return((a & 0x3FFF) | ((a >> 1) & 0x4000));
}

int15_t s15_getOvf(int15_t a)
{
    // Check for overflow
    switch (a & 0xC000)
    {
        case 0x4000:
         // Positive overflow
            return 1;
        break;
        case 0x8000:
        // Negative overflow
            return -1;
        break;
        default:
            return 0;
        break;
    }
}

int15_t s15_se(int15_t a)
{
    // Copy bit 15 to bit 16
    return((a & 0x7FFF) | ((a << 1) & 0x8000));
}