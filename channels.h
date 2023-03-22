#ifndef CHANNELS_H
#define CHANNELS_H

/* Read CH30 input */
#define CH30_Read() 0

/* Read CH32 input */
#define CH32_Read() 0

extern volatile int15_t CH11;

extern volatile int15_t CH12;

extern volatile int15_t CH14;

extern volatile int15_t CH33;

/* Write 15 bit string to display relays */
#define Dis_Write(x) 

#endif