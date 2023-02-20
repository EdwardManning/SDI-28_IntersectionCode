#include "./Driver.h"

Driver::Driver()
{
    //do nothing 
}

Driver::Driver(DriverType driver_type_)
{
    SWERRINT(driver_type_);
}

std::string Driver::name()
{
    return my_name;
}

DriverType Driver::driverType()
{
    return my_driverType;
}

float Driver::modifier()
{
    return my_modifier;
}

float Driver::maximumAcceleration()
{
    return my_maximumAcceleration;
}

float Driver::reactionTime()
{
    return my_reactionTime;
}