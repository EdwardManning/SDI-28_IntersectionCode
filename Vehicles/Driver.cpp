#include "./Driver.h"

Driver::Driver()
{
    //do nothing 
}

Driver::Driver(DriverType driver_type_)
{
    SWERRINT(driver_type_);
}

float Driver::generateComfortableAcceleration()
{
    std::random_device root;
    std::mt19937 acceleration(root());
    switch(my_driverType)
    {
        case(NULL_DRIVER):
        {
            return 3;
        }
            break;
        case(CALM):
        {
            std::uniform_real_distribution<> distribution(2, 3);
            return distribution(acceleration);
        }
            break;
        case(NORMAL):
        {
            std::uniform_real_distribution<> distribution((3 - 0.5), (3 + 0.5));
            return distribution(acceleration);
        }
            break;
        case(AGGRESSIVE):
        {
            std::uniform_real_distribution<> distribution(3, 4);
            return distribution(acceleration);
        }
            break;
        default:SWERRINT(my_driverType);
    }
    return 3;
}

float Driver::generateComfortableDeceleration()
{
    std::random_device root;
    std::mt19937 acceleration(root());
    switch(my_driverType)
    {
        case(NULL_DRIVER):
        {
            return 3;
        }
            break;
        case(CALM):
        {
            std::uniform_real_distribution<> distribution(2, 3);
            return distribution(acceleration);
        }
            break;
        case(NORMAL):
        {
            std::uniform_real_distribution<> distribution((3 - 0.5), (3 + 0.5));
            return distribution(acceleration);
        }
            break;
        case(AGGRESSIVE):
        {
            std::uniform_real_distribution<> distribution(3, 4);
            return distribution(acceleration);
        }
            break;
        default:SWERRINT(my_driverType);
    }
    return 3;
}

float Driver::generateReactionTime()
{
    std::random_device root;
    std::mt19937 reaction_time(root());
    switch(my_driverType)
    {
        case(NULL_DRIVER):
        {
            return 0;
        }
            break;
        case(CALM):
        {
            std::uniform_real_distribution<> distribution(0.25, 0.33);
            return distribution(reaction_time);
        }
            break;
        case(NORMAL):
        {
            std::uniform_real_distribution<> distribution((0.2), (0.3));
            return distribution(reaction_time);
        }
            break;
        case(AGGRESSIVE):
        {
            std::uniform_real_distribution<> distribution(0.18, 0.25);
            return distribution(reaction_time);
        }
            break;
        default:SWERRINT(my_driverType);
    }
    return 0;
}

float Driver::setSlowingDistance()
{
    if(my_driverType == CALM)
    {
        return 3.5 * my_minFollowingDistance;
    }
    else if (my_driverType == AGGRESSIVE)
    {
        return 2 * my_minFollowingDistance;
    }
    else
    {
        return 3 * my_minFollowingDistance;
    }
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

float Driver::reactionTime()
{
    return my_reactionTime;
}

float Driver::slowingDistance()
{
    return my_slowingDistance;
}

float Driver::comfortableAcceleration()
{
    return my_comfortableAcceleration;
}

float Driver::comfortableDeceleration()
{
    return my_comfortableDeceleration;
}

float Driver::minimumFollowingDistance()
{
    return my_minFollowingDistance;
}

float Driver::minimumStoppingDistance()
{
    return my_minStopDistance;
}