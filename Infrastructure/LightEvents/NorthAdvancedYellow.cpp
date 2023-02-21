#include "./NorthAdvancedYellow.h"

NorthAdvancedYellow::NorthAdvancedYellow(float duration_)
{
    my_event = NORTH_ADVANCED_YELLOW;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = NORTH_GREEN_LIGHT + NORTH_ADVANCED_YELLOW_LIGHT;
    my_duration = duration_;
    my_nextEvent = NORTH_GREEN;
}