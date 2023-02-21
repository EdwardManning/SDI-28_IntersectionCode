#pragma once
#include "../Common/CommonDefs.h"
#include "./LightEvents/CommonLightEvents.h"

class TrafficLight
{
public:
    TrafficLight(bool smart_ = false);
    bool makeStep();
    void changeState(lightState state_, bool add_ = ADD);
    void changeEvent(lightEventState event_);
    void changeEvent(lightEventState event_, float duration_);
    uint16 state();
    lightEventState currentEvent();
    lightEventState nextEvent();
    float timer();
private:
    void cycle();
    bool my_type;
    float my_timer;
    lightEventState my_currentEventState;
    lightEventState my_nextEventState;
    LightEvent* my_currentEvent;
    uint16 my_currentState;
};