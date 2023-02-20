#include "./HumanDriver.h"

HumanDriver::HumanDriver(DriverType driver_type)
{
    my_driverType = driver_type;
    my_name = DRIVER_TYPE_STR[my_driverType] + " " + DRIVER_STR;
    my_modifier = DRIVER_TYPE_MODIFIER[my_driverType];

    //change the following to be random once we add in reaction times
    my_reactionTime = 0;
}