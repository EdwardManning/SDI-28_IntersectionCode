#include "./EWGreen.h"

EWGreen::EWGreen(float duration_)
{
    my_event = EW_GREEN;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = EAST_GREEN_LIGHT + WEST_GREEN_LIGHT;
    my_duration = duration_;
    my_nextEvent = EW_YELLOW;
}