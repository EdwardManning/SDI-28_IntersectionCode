#include "./NSAdvancedRed.h"

NSAdvancedRed::NSAdvancedRed(float duration_)
{
    my_event = NS_ADVANCED_RED;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = 0;
    my_duration = duration_;
    my_nextEvent = NS_GREEN;
}