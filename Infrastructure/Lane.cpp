#include "Lane.h"

//Function definitions for Lane.h

Lane::Lane()
{
    //leave empty for initialization of blank array
}

//the following constructor should never be used
Lane::Lane(uint8 number_, direction direction_, path path_)
{
    SWERRINT(number_<<5 + direction_<<3 + path_);
}

uint32 Lane::vehicleAtIndex(uint32 index_)
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

uint32 Lane::indexOfVehicle(uint32 vehicle_number_)
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

bool Lane::inLane(uint32 vehicle_number_)
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

void Lane::addToLane(uint32 vehicle_number_)
{
    my_vehicles.insert(my_vehicles.begin(), vehicle_number_);
}

void Lane::addToLane(uint32 vehicle_number_, uint32 index_)
{
    my_vehicles.insert(my_vehicles.begin() + index_, vehicle_number_);
}

void Lane::removeFromLane()
{
    my_vehicles.pop_back();
}

bool Lane::removeFromLane(uint32 vehicle_number_)
{
    if (inLane(vehicle_number_))
    {
        my_vehicles.erase(my_vehicles.begin() + indexOfVehicle(vehicle_number_));
        return true;
    }
    return false;
}

//The following functions are used to access the values of the 
//protected members. They all return the variable of the same name
//and do nothing else.
float* Lane::startingPosition()
{
    return my_startingPosition;
}

float* Lane::endingPosition()
{
    return my_endingPosition;
}

float Lane::centerLine()
{
    return my_centerLine;
}

uint8 Lane::width()
{
    return my_width;
}

uint8 Lane::length()
{
    return my_length;
}

std::string Lane::name()
{
    return my_name;
}

uint8 Lane::number()
{
    return my_number;
}

direction Lane::laneDirection()
{
    return my_laneDirection;
}

path Lane::lanePath()
{
    return my_lanePath;
}

bool Lane::laneType()
{
    return my_laneType;
}

uint8 Lane::speedLimit()
{
    return my_speedLimit;
}

int8* Lane::unitVector()
{
    return my_unitVector;
}

uint32 Lane::numberOfVehicles()
{
    return my_vehicles.size();
}