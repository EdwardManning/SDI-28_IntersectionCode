#include "./EWYellow.h"

EWYellow::EWYellow(float duration_)
{
    my_event = EW_YELLOW;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = EAST_YELLOW_LIGHT + WEST_YELLOW_LIGHT;
    my_duration = duration_;
    my_nextEvent = NS_MUTUAL_RED;
}