#include "./EWAdvancedGreen.h"

EWAdvancedGreen::EWAdvancedGreen(float duration_)
{
    my_event = EW_ADVANCED_GREEN;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = EAST_ADVANCED_GREEN_LIGHT + WEST_ADVANCED_GREEN_LIGHT;
    my_duration = duration_;
    my_nextEvent = EW_ADVANCED_YELLOW;
}