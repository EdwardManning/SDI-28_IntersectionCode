#include "Road.h"

//Function definitions for Road.h

Road::Road()
{
    //leave empty for initialization of blank array
}

/*
*   Name: getLane
*
*   Description: Returns a lane based on the lane_number given.
*
*   Input: lane_number_ -> The number of the lane requested.
*
*   Output: The lane requested.
*
*/
Lane* Road::getLane(uint8 lane_number_)
{
    if(lane_number_ < my_totalLanes)
    {
        return my_laneList[lane_number_];
    }
    else
    {
        SWERRINT(lane_number_);
    }
    return my_laneList[0]; //only gets hit if swerr
}

//The following functions are used to access the values of the 
//protected members. They all return the variable of the same name
// and do nothing else.
uint8 Road::totalLanes()
{
    return my_totalLanes;
}

uint16 Road::startingPosition()
{
    return my_startingPosition;
}

uint16 Road::endingPosition()
{
    return my_endingPosition;
}

std::string Road::name()
{
    return my_name;
}

direction Road::roadDirection()
{
    return my_direction;
}

uint8 Road::speedLimit()
{
    return my_speedLimit;
}