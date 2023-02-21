#include "./NSYellow.h"

NSYellow::NSYellow(float duration_)
{
    my_event = NS_YELLOW;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = NORTH_YELLOW_LIGHT + SOUTH_YELLOW_LIGHT;
    my_duration = duration_;
    my_nextEvent = EW_MUTUAL_RED;
}