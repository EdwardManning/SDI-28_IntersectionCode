#include "./NSMutualRed.h"

NSMutualRed::NSMutualRed(float duration_)
{
    my_event = NS_MUTUAL_RED;
    my_name = LIGHT_EVENT_STATE_STR[my_event];
    my_lightState = 0;
    my_duration = duration_;
    if(intersection_params.n_advanced_green && intersection_params.s_advanced_green)
    {
        my_nextEvent = NS_ADVANCED_GREEN;
    } 
    else if(intersection_params.n_advanced_green)
    {
        my_nextEvent = NORTH_ADVANCED_GREEN;
    } 
    else if(intersection_params.s_advanced_green)
    {
        my_nextEvent = SOUTH_ADVANCED_GREEN;
    }
    else
    {
        my_nextEvent = NS_GREEN;
    }
}
