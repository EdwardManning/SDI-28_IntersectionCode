#pragma once
#include <vector>
#include "../../Common/CommonDefs.h"

class LightEvent
{
public:
    LightEvent();
    LightEvent(float duration_);
    std::string name();
    lightEventState eventState();
    lightEventState nextEvent();
    uint16 lightState();
    float duration();
protected:
    std::string my_name;
    lightEventState my_event;
    lightEventState my_nextEvent;
    uint16 my_lightState;
    float my_duration;
private:
};
