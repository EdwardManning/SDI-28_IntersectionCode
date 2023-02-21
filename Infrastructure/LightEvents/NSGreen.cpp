#include "./NSGreen.h"

NSGreen::NSGreen(float duration_)
{
    my_event = NS_GREEN;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = NORTH_GREEN_LIGHT + SOUTH_GREEN_LIGHT;
    my_duration = duration_;
    my_nextEvent = NS_YELLOW;
}