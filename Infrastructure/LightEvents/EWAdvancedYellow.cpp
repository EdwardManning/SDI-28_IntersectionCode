#include "./EWAdvancedYellow.h"

EWAdvancedYellow::EWAdvancedYellow(float duration_)
{
    my_event = EW_ADVANCED_YELLOW;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = EAST_ADVANCED_YELLOW_LIGHT + WEST_ADVANCED_YELLOW_LIGHT;
    my_duration = duration_;
    my_nextEvent = EW_ADVANCED_RED;
}