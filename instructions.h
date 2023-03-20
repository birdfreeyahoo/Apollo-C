#ifndef INSTRUCTIONS_H
#define INSTRUCTIONS_H

#define INHINT() (__asm__("INHINT"))

/* Disable interrupts */
#define Int_Disable() INHINT()

#endif