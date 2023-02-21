#include "./SouthAdvancedGreen.h"

SouthAdvancedGreen::SouthAdvancedGreen(float duration_)
{
    my_event = SOUTH_ADVANCED_GREEN;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = SOUTH_GREEN_LIGHT + SOUTH_ADVANCED_GREEN_LIGHT;
    my_duration = duration_;
    my_nextEvent = SOUTH_ADVANCED_YELLOW;
}