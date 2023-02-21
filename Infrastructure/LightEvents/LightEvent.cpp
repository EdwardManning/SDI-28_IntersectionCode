#include "LightEvent.h"

LightEvent::LightEvent()
{
    //do nothing
}

LightEvent::LightEvent(float duration_)
{
    SWERRFLOAT(duration_);
}

std::string LightEvent::name()
{
    return my_name;
}

lightEventState LightEvent::eventState()
{
    return my_event;
}

lightEventState LightEvent::nextEvent()
{
    return my_nextEvent;
}

uint16 LightEvent::lightState()
{
    return my_lightState;
}

float LightEvent::duration()
{
    return my_duration;
}