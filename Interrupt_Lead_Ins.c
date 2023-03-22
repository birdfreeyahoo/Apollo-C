#include "defines.h"
#include "instructions.h"
#include "tmr.h"
#include "util.h"
#include "T4_Program.h"

// ISR start at 4000 octal
// There are 4 words per ISR

// GOJAM
void GOJAM_ISR(void)
{
    Int_Disable();

    // Loading Banks

    // TODO: Goto Goprog
}

// T6RUPT
void T6RUPT_ISR(void)
{
    // TODO: Goto T6 handler
}

// T5RUPT
void T5RUPT_ISR(void)
{
    // Load timer 5 value
    int16_t timer5 = Tmr_GetTime5();

    if(timer5 >= TMR_HALF_SECOND)
    {
        // TODO: Figure out scheduler call
    }

    // TODO: Goto T5 handler
}

// T3RUPT
void T3RUPT_ISR(void)
{
    // TODO: Goto T3 handler
}

// T4RUPT
void T4RUPT_ISR(void)
{
    T4_Program();
}

// KEYRUPT1
void KEYRUPT1_ISR(void)
{
    // TODO: Goto Key1 handler
}

// KEYRUPT2
void KEYRUPT2_ISR(void)
{
    // TODO: Goto Key2 handler
}

// UPRUPT
void UPRUPT_ISR(void)
{
    // TODO: Goto Up handler
}

// DNRUPT
void DNRUPT_ISR(void)
{
    // TODO: Goto transmit handler
}

// RADAR RUPT
void RADARRUPT_ISR(void)
{
    // Go to radar handler
}

// Hand control rupt
void HCRUPT_ISR(void)
{
    // Not used
}

