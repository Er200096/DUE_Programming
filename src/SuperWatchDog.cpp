#include "SuperWatchDog.h"
#include <Arduino.h>

void setup_watchdog(){
    watchdogEnable(1000);
}

int StartWatchDog(){
    return 1;
}
