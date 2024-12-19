
#include "msp.h"
#include "Clock.h"
#include <stdio.h>
#include "module.h"
#include <stdio.h>




void main(void)
{
    Clock_Init48MHz();
    LaunchPad_Init();
    SysTick_Init();   // change log  < - //SysTick_Init(48000, 2);
    Reflectance_Init();
    Motor_Init();
    Blinker_Init();
    Odometry_Init(0,0,NORTH);
    Tachometer_Init();
    TimerA1_Init(&UpdatePosition,20000); // every 40ms
    EnableInterrupts();
    start_fsm();




    printf("while finished and terminated main()\n");
}



