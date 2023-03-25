#include "Interrupt_Lead_Ins.h"
#include "registers.h"
#include <time.h>
#include <signal.h>

static volatile int keepRunning = 1;


// TODO: Move to own file
int16_t TIME4;

void exitHandler(int _d)
{
    keepRunning = 0;
}

static clock_t getMilliseconds(void)
{
    return clock() / (CLOCKS_PER_SEC / 1000);
}

static void task10Ms(void)
{
    if(TIME4 >= 16384)
    {
        TIME4 = 0;
        T4RUPT_ISR();
    }

    TIME4++;
}

int main(int argc, char** argv)
{
    signal(SIGINT, exitHandler);
    
    clock_t lastTime = getMilliseconds();

    while(keepRunning)
    {
        // Wait 10 millsecond
        while(getMilliseconds() - lastTime < 10);

        // Get time before executing task to have execution time as part of wait time
        lastTime = getMilliseconds();

        task10Ms();
    }
}