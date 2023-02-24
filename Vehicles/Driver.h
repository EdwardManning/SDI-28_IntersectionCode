# pragma once
#include "../Common/CommonDefs.h"
#include <random>

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
    float reactionTime();
    float comfortableAcceleration();
    float comfortableDeceleration();
    float minimumStoppingDistance();
    float minimumFollowingDistance();
protected:
    float generateComfortableDeceleration();
    float generateComfortableAcceleration();
    float generateReactionTime();
    std::string my_name;
    DriverType my_driverType;
    float my_modifier;
    float my_reactionTime;
    float my_minFollowingDistance;
    float my_minStopDistance;
    float my_comfortableDeceleration;
    float my_comfortableAcceleration;
private:
};