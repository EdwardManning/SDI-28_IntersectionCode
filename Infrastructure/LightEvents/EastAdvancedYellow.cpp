#include "./EastAdvancedYellow.h"

EastAdvancedYellow::EastAdvancedYellow(float duration_)
{
    my_event = EAST_ADVANCED_YELLOW;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = EAST_GREEN_LIGHT + EAST_ADVANCED_YELLOW_LIGHT;
    my_duration = duration_;
    my_nextEvent = EAST_GREEN;
}