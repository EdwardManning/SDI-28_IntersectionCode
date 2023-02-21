#include "./SouthAdvancedYellow.h"

SouthAdvancedYellow::SouthAdvancedYellow(float duration_)
{
    my_event = SOUTH_ADVANCED_YELLOW;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = SOUTH_GREEN_LIGHT + SOUTH_ADVANCED_YELLOW_LIGHT;
    my_duration = duration_;
    my_nextEvent = SOUTH_GREEN;
}