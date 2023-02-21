#include "./SouthGreen.h"

SouthGreen::SouthGreen(float duration_)
{
    my_event = SOUTH_GREEN;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = SOUTH_GREEN_LIGHT;
    my_duration = duration_;
    my_nextEvent = NS_GREEN;
}