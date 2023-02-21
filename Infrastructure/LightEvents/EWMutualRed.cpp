#include "./EWMutualRed.h"

EWMutualRed::EWMutualRed(float duration_)
{
    my_event = EW_MUTUAL_RED;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = 0;
    my_duration = duration_;
    if(intersection_params.e_advanced_green && intersection_params.w_advanced_green)
    {
        my_nextEvent = EW_ADVANCED_GREEN;
    } 
    else if(intersection_params.e_advanced_green)
    {
        my_nextEvent = EAST_ADVANCED_GREEN;
    } 
    else if(intersection_params.w_advanced_green)
    {
        my_nextEvent = WEST_ADVANCED_GREEN;
    }
    else
    {
        my_nextEvent = EW_GREEN;
    }
}