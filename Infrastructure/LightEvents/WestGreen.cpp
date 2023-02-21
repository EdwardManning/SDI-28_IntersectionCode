#include "./WestGreen.h"

WestGreen::WestGreen(float duration_)
{
    my_event = WEST_GREEN;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = WEST_GREEN_LIGHT;
    my_duration = duration_;
    my_nextEvent = EW_GREEN;
}