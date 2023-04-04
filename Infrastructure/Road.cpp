#include "Road.h"

//Function definitions for Road.h

Road::Road()
{
    //leave empty for initialization of blank array
}

uint32 Road::vehicleAtIndex(uint32 index_)
{
    try
    {
        if(index_ < my_vehicles.size() && index_ >= 0)
        {
            return my_vehicles[index_];
        }
        else
        {
            throw(std::out_of_range("Out of Vehicle List Bounds"));
        }
    }
    catch (const std::out_of_range &Out_of_Range)
    {
        //hard SWERR
        SWERRINT(index_);
        SWERRSTR(Out_of_Range.what());
        throw;
    }
}

uint32 Road::indexOfVehicle(uint32 vehicle_number_)
{
    if (my_vehicles.size() > 0)
    {
        for(uint32 i = 0; i < my_vehicles.size(); i++)
        {
            if (vehicle_number_ == my_vehicles[i])
            {
                return i;
            }
        }
    }
    SWERRINT(-1);
    return 0;
}

bool Road::inRoad(uint32 vehicle_number_)
{
    if (my_vehicles.size() > 0)
    {
        for(uint32 i = 0; i < my_vehicles.size(); i++)
        {
            if (vehicle_number_ == my_vehicles[i])
            {
                return true;
            }
        }
    }
    return false;
}

void Road::addToRoad(uint32 vehicle_number_)
{
    my_vehicles.insert(my_vehicles.begin(), vehicle_number_);
}

void Road::addToRoad(uint32 vehicle_number_, uint32 index_)
{
    my_vehicles.insert(my_vehicles.begin() + index_, vehicle_number_);
}

void Road::removeFromRoad()
{
    my_vehicles.pop_back();
}

bool Road::removeFromRoad(uint32 vehicle_number_)
{
    if (inRoad(vehicle_number_))
    {
        my_vehicles.erase(my_vehicles.begin() + indexOfVehicle(vehicle_number_));
        return true;
    }
    return false;
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

/*
*   Name: correspondingExit
*
*   Description: Overridden by derived classes. Should not be used.
*
*/
direction Road::correspondingExit(path path_)
{
    SWERRINT(path_);
    return NORTH;
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

uint32 Road::numberOfVehicles()
{
    return my_vehicles.size();
}