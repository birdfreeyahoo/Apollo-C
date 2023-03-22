#ifndef EXECUTIVE_H
#define EXECUTIVE_H

#include "defines.h"

/* Function pointer type for job */
typedef void (*JobFunction)(void);

/* Queues new job request which does not need VAC area */
void Exec_JobRequest_Novac(JobFunction job, int15_t priority);

#endif