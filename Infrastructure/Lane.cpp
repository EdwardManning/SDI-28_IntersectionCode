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