#include "./SelfDriver.h"

SelfDriver::SelfDriver()
{
    my_driverType = NULL_DRIVER;
    my_name = DRIVER_TYPE_STR[my_driverType] + " " + DRIVER_STR;
    my_modifier = DRIVER_TYPE_MODIFIER[my_driverType];

    my_comfortableAcceleration = generateComfortableAcceleration();
    my_comfortableDeceleration = generateComfortableDeceleration();
    
    //change based on temperment
    my_minStopDistance = 3;
    my_minFollowingDistance = vehicle_params.vehicle_length;

    my_slowingDistance = setSlowingDistance();

    my_reactionTime = generateReactionTime();
}