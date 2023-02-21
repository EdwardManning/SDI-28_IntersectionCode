#include "./EastAdvancedGreen.h"

EastAdvancedGreen::EastAdvancedGreen(float duration_)
{
    my_event = EAST_ADVANCED_GREEN;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = EAST_GREEN_LIGHT + EAST_ADVANCED_GREEN_LIGHT;
    my_duration = duration_;
    my_nextEvent = EAST_ADVANCED_YELLOW;
}