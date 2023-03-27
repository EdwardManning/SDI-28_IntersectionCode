#pragma once
#include "../Common/CommonDefs.h"
#include "./LightEvents/CommonLightEvents.h"

class TrafficLight
{
public:
    TrafficLight(bool smart_ = false);
    bool makeStep();
    lightColour currentLightColour(direction direction_);
    lightColour previousLightColour(direction direction_);
    lightColour advancedLightColour(direction direction_);
    void changeState(lightState state_, bool add_ = ADD);
    void changeEvent(lightEventState event_);
    void changeEvent(lightEventState event_, float duration_);
    uint16 state();
    uint16 previousState();
    lightEventState currentEvent();
    lightEventState nextEvent();
    float timer();
    float currentEventDuration();
private:
    void cycle();
    bool my_type;
    float my_timer;
    lightEventState my_currentEventState;
    lightEventState my_nextEventState;
    LightEvent* my_currentEvent;
    uint16 my_currentState;
    uint16 my_previousState;
};