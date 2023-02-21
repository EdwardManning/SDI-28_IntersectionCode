#include "./EWAdvancedRed.h"

EWAdvancedRed::EWAdvancedRed(float duration_)
{
    my_event = EW_ADVANCED_RED;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = 0;
    my_duration = duration_;
    my_nextEvent = EW_GREEN;
}