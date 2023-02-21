#include "./WestAdvancedYellow.h"

WestAdvancedYellow::WestAdvancedYellow(float duration_)
{
    my_event = WEST_ADVANCED_YELLOW;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = WEST_GREEN_LIGHT + WEST_ADVANCED_YELLOW_LIGHT;
    my_duration = duration_;
    my_nextEvent = WEST_GREEN;
}