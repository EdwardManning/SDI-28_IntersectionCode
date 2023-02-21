#include "./NorthAdvancedGreen.h"

NorthAdvancedGreen::NorthAdvancedGreen(float duration_)
{
    my_event = NORTH_ADVANCED_GREEN;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = NORTH_GREEN_LIGHT + NORTH_ADVANCED_GREEN_LIGHT;
    my_duration = duration_;
    my_nextEvent = NORTH_ADVANCED_YELLOW;
}