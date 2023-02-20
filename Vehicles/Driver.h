# pragma once
#include "../Common/CommonDefs.h"

class Driver
{
public:
    Driver();
    Driver(DriverType driver_type_);
    //add in driver probability functions
    //i.e. probability of running yellow

    std::string name();
    DriverType driverType();
    float modifier();
    float maximumAcceleration();
    float reactionTime();
protected:
    std::string my_name;
    DriverType my_driverType;
    float my_modifier;
    float my_maximumAcceleration;
    float my_reactionTime;
private:
};