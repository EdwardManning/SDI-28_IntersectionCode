#include "./NSAdvancedGreen.h"

NSAdvancedGreen::NSAdvancedGreen(float duration_)
{
    my_event = NS_ADVANCED_GREEN;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = NORTH_ADVANCED_GREEN_LIGHT + SOUTH_ADVANCED_GREEN_LIGHT;
    my_duration = duration_;
    my_nextEvent = NS_ADVANCED_YELLOW;
}