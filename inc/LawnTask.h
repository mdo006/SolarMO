#ifndef LawnTask_h
#define LawnTask_h

#include <Arduino.h>
#include <Motors.h>

enum LawnState {INIT_7, FWD3, LEFT, RIGHT, END, UTURN} Lawn_State;

Lawn_Init {
    Lawn_State = INIT_7;
}

Lawn_Tick() {
    
}


#endif
