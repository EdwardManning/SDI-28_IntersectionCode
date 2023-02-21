#include "./NSAdvancedYellow.h"

NSAdvancedYellow::NSAdvancedYellow(float duration_)
{
    my_event = NS_ADVANCED_YELLOW;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = NORTH_ADVANCED_YELLOW_LIGHT + SOUTH_ADVANCED_YELLOW_LIGHT;
    my_duration = duration_;
    my_nextEvent = NS_ADVANCED_RED;
}