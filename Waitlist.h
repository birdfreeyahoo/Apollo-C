#ifndef WAITLIST_H
#define WAITLIST_H

#include "defines.h"

typedef void (*Waitlist_Callback_t)(void);

void Waitlist_Waitlist(int16_t time, Waitlist_Callback_t callback);
void Waitlist_VarDelay(int16_t time, Waitlist_Callback_t callback);
void Waitlist_TaskOver(void);

#endif