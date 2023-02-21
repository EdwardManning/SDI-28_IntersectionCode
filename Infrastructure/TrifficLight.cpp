#include "TrafficLight.h"

TrafficLight::TrafficLight(bool smart_)
{
    my_type = smart_;
    my_timer = 0;
    my_currentEventState = NS_MUTUAL_RED;

    changeEvent(my_currentEventState);

}

bool TrafficLight::makeStep()
{
    my_timer = my_timer + simulation_params.time_step;
    if (my_timer > my_currentEvent->duration())
    {
        cycle();
        return true;
    }
    return false;
}

void TrafficLight::cycle()
{
    //this will eventually be used with the traffic light to 
    //set arbitrary durations for changeEvent (not just the basic one)
    //this will allow for early light changes depending on traffic
    changeEvent(my_nextEventState);
}

void TrafficLight::changeState(lightState state_, bool add_)
{
    if (add_)
    {
        if (my_currentState & state_)
        {
            SWERRINT(state_);
        }
        else
        {
            my_currentState += state_;
        }
    }
    else
    {
        if(my_currentState & state_)
        {
            my_currentState -= state_;
        }
        else
        {
            SWERRINT(state_);
        }
    }
}

void TrafficLight::changeEvent(lightEventState event_)
{
    switch(event_)
    {
        case(NORTH_ADVANCED_GREEN):
        {
            my_currentEvent = new NorthAdvancedGreen(intersection_params.ns_advanced_green_time);
        }
            break;
        case(SOUTH_ADVANCED_GREEN):
        {
            my_currentEvent = new SouthAdvancedGreen(intersection_params.ns_advanced_green_time);
        }
            break;
        case(EAST_ADVANCED_GREEN):
        {
            my_currentEvent = new EastAdvancedGreen(intersection_params.ew_advanced_green_time);
        }
            break;
        case(WEST_ADVANCED_GREEN):
        {
            my_currentEvent = new WestAdvancedGreen(intersection_params.ew_advanced_green_time);
        }
            break;
        case(NORTH_ADVANCED_YELLOW):
        {
            my_currentEvent = new NorthAdvancedYellow(intersection_params.ns_advanced_yellow_time);
        }
            break;
        case(SOUTH_ADVANCED_YELLOW):
        {
            my_currentEvent = new SouthAdvancedYellow(intersection_params.ns_advanced_yellow_time);
        }
            break;
        case(EAST_ADVANCED_YELLOW):
        {
            my_currentEvent = new EastAdvancedYellow(intersection_params.ew_advanced_yellow_time);
        }
            break;
        case(WEST_ADVANCED_YELLOW):
        {
            my_currentEvent = new WestAdvancedYellow(intersection_params.ew_advanced_yellow_time);
        }
            break;
        case(NS_ADVANCED_GREEN):
        {
            my_currentEvent = new NSAdvancedGreen(intersection_params.ns_advanced_green_time);
        }
            break;
        case(EW_ADVANCED_GREEN):
        {
            my_currentEvent = new EWAdvancedGreen(intersection_params.ns_advanced_green_time);
        }
            break;
        
        case(NS_ADVANCED_RED):
        {
            my_currentEvent = new NSAdvancedRed(intersection_params.ns_advanced_red_time);
        }
            break;
        
        case(EW_ADVANCED_RED):
        {
            my_currentEvent = new EWAdvancedRed(intersection_params.ew_advanced_red_time);
        }
            break;
        case(NS_ADVANCED_YELLOW):
        {
            my_currentEvent = new NSAdvancedYellow(intersection_params.ns_advanced_yellow_time);
        }
            break;
        case(EW_ADVANCED_YELLOW):
        {
            my_currentEvent = new EWAdvancedYellow(intersection_params.ew_advanced_yellow_time);
        }
            break;
        case(NS_GREEN):
        {
            my_currentEvent = new NSGreen(intersection_params.ns_green_time);
        }
            break;
        case(EW_GREEN):
        {
            my_currentEvent = new EWGreen(intersection_params.ew_green_time);
        }
            break;
        case(NS_YELLOW):
        {
            my_currentEvent = new NSYellow(intersection_params.ns_yellow_time);
        }
            break;
        case(EW_YELLOW):
        {
            my_currentEvent = new EWYellow(intersection_params.ns_yellow_time);
        }
            break;
        case(NS_MUTUAL_RED):
        {
            my_currentEvent = new NSMutualRed(intersection_params.ns_red_overlap_time);
        }
            break;
        case(EW_MUTUAL_RED):
        {
            my_currentEvent = new EWMutualRed(intersection_params.ew_red_overlap_time);
        }
            break;
        case(NORTH_GREEN):
        {
            my_currentEvent = new NorthGreen(intersection_params.ns_advanced_red_time);
        }
            break;
        case(SOUTH_GREEN):
        {
            my_currentEvent = new SouthGreen(intersection_params.ns_advanced_red_time);
        }
            break;
        case(EAST_GREEN):
        {
            my_currentEvent = new EastGreen(intersection_params.ew_advanced_red_time);
        }
            break;
        case(WEST_GREEN):
        {
            my_currentEvent = new WestGreen(intersection_params.ew_advanced_red_time);
        }
            break;
        default: 
        {
            SWERRINT(event_);
            //return to NS mutual red since it is the start of the cycle
            //this is in hopes of breaking any possible loop situations 
            //or impossible paths
            my_currentEvent = new NSMutualRed(intersection_params.ns_red_overlap_time);
        }
    }

    my_currentEventState = my_currentEvent->eventState();
    my_currentState = my_currentEvent->lightState();
    my_nextEventState = my_currentEvent->nextEvent();
    my_timer = 0;
}

void TrafficLight::changeEvent(lightEventState event_, float duration_)
{
    if (duration_ < 0)
    {
        SWERRFLOAT(duration_);
        changeEvent(event_);
    }
    switch(event_)
    {
        case(NORTH_ADVANCED_GREEN):
        {
            my_currentEvent = new NorthAdvancedGreen(duration_);
        }
            break;
        case(SOUTH_ADVANCED_GREEN):
        {
            my_currentEvent = new SouthAdvancedGreen(duration_);
        }
            break;
        case(EAST_ADVANCED_GREEN):
        {
            my_currentEvent = new EastAdvancedGreen(duration_);
        }
            break;
        case(WEST_ADVANCED_GREEN):
        {
            my_currentEvent = new WestAdvancedGreen(duration_);
        }
            break;
        case(NORTH_ADVANCED_YELLOW):
        {
            my_currentEvent = new NorthAdvancedYellow(duration_);
        }
            break;
        case(SOUTH_ADVANCED_YELLOW):
        {
            my_currentEvent = new SouthAdvancedYellow(duration_);
        }
            break;
        case(EAST_ADVANCED_YELLOW):
        {
            my_currentEvent = new EastAdvancedYellow(duration_);
        }
            break;
        case(WEST_ADVANCED_YELLOW):
        {
            my_currentEvent = new WestAdvancedYellow(duration_);
        }
            break;
        case(NS_ADVANCED_GREEN):
        {
            my_currentEvent = new NSAdvancedGreen(duration_);
        }
            break;
        case(EW_ADVANCED_GREEN):
        {
            my_currentEvent = new EWAdvancedGreen(duration_);
        }
            break;
        
        case(NS_ADVANCED_RED):
        {
            my_currentEvent = new NSAdvancedRed(duration_);
        }
            break;
        
        case(EW_ADVANCED_RED):
        {
            my_currentEvent = new EWAdvancedRed(duration_);
        }
            break;
        case(NS_ADVANCED_YELLOW):
        {
            my_currentEvent = new NSAdvancedYellow(duration_);
        }
            break;
        case(EW_ADVANCED_YELLOW):
        {
            my_currentEvent = new EWAdvancedYellow(duration_);
        }
            break;
        case(NS_GREEN):
        {
            my_currentEvent = new NSGreen(duration_);
        }
            break;
        case(EW_GREEN):
        {
            my_currentEvent = new EWGreen(duration_);
        }
            break;
        case(NS_YELLOW):
        {
            my_currentEvent = new NSYellow(duration_);
        }
            break;
        case(EW_YELLOW):
        {
            my_currentEvent = new EWYellow(duration_);
        }
            break;
        case(NS_MUTUAL_RED):
        {
            my_currentEvent = new NSMutualRed(duration_);
        }
            break;
        case(EW_MUTUAL_RED):
        {
            my_currentEvent = new EWMutualRed(duration_);
        }
            break;
        case(NORTH_GREEN):
        {
            my_currentEvent = new NorthGreen(duration_);
        }
            break;
        case(SOUTH_GREEN):
        {
            my_currentEvent = new SouthGreen(duration_);
        }
            break;
        case(EAST_GREEN):
        {
            my_currentEvent = new EastGreen(duration_);
        }
            break;
        case(WEST_GREEN):
        {
            my_currentEvent = new WestGreen(duration_);
        }
            break;
        default: 
        {
            SWERRINT(event_);
            //return to NS mutual red since it is the start of the cycle
            //this is in hopes of breaking any possible loop situations 
            //or impossible paths
            my_currentEvent = new NSMutualRed(duration_);
        }
    }

    my_currentEventState = my_currentEvent->eventState();
    my_currentState = my_currentEvent->lightState();
    my_nextEventState = my_currentEvent->nextEvent();
    my_timer = 0;
}

uint16 TrafficLight::state()
{
    return my_currentState;
}

lightEventState TrafficLight::currentEvent()
{
    return my_currentEventState;
}

lightEventState TrafficLight::nextEvent()
{
    return my_nextEventState;
}

float TrafficLight::timer()
{
    return my_timer;
}