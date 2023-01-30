#include "Vehicle.h"

//Function definitions for Vehicle.h

Vehicle::Vehicle()
{
    //leave blank for initialization
}

//the following should never be used
Vehicle::Vehicle(uint16 number_, path path_, Lane* lane_, DriverType driver_type_)
{
    SWERRINT(number_<<5 + path_<<4 + driver_type_);
}

/*
*   Name: drive
*
*   Description: Drives the vehicle.
*
*   Input: N/A
*
*   Output: N/A
*
*/
void Vehicle::drive()
{
    if (my_state & DRIVING) //if driving
    {
        my_currentPosition[x] += my_currentVelocity[x] * simulation_params.time_step;
        my_currentPosition[y] += my_currentVelocity[y] * simulation_params.time_step;
    }
    else
    {
        SWERRINT(my_state);
    }
}

/*
*   Name: accelerate
*
*   Description: Accelerates the vehicle.
*
*   Input: N/A
*
*   Output: N/A
*
*/
void Vehicle::accelerate()
{
    if(my_state & ACCELERATING) //if accelerating
    {
        my_currentVelocity[x] += my_currentAcceleration[x] * simulation_params.time_step;
        my_currentVelocity[y] += my_currentAcceleration[y] * simulation_params.time_step;
    } 
    else if (my_state & DECELERATING) //if decelerating
    {
        my_currentVelocity[x] -= my_currentAcceleration[x] * simulation_params.time_step;
        my_currentVelocity[y] -= my_currentAcceleration[y] * simulation_params.time_step;
    }
    else //if neither accelerating or decelerating
    {
        SWERRINT(my_state);
    }
}

/*
*   Name: changeState
*
*   Description: Adds or removes a state from the vehicle's my_state.
*
*   Input: state_ -> The state being added or removed.
*          adding -> Boolean stating weather the state is being added or removed.
*         
*   Output: N/A
*
*/
void Vehicle::changeState(state state_, const bool adding_)
{
    if (adding_)
    {
        if (my_state & state_) //state is present, cannot be added
        {
            SWERRINT(state_);
        }
        else //state is not present, can be added
        {
            my_state += state_; 
        }
    }
    else
    {
        if(my_state & state_) //state is present, can be removed
        {
            my_state -= state_;
        }
        else //state is not present, cannot be removed
        {
            SWERRINT(state_);
        }
    }
}

//The following functions are used to access the values of the 
//protected members. They all return the variable of the same name
//and do nothing else.
float* Vehicle::currentPosition()
{
    return my_currentPosition;
}

float* Vehicle::currentVelocity()
{
    return my_currentVelocity;
}

float* Vehicle::currentAcceleration()
{
    return my_currentAcceleration;
}

uint16 Vehicle::number()
{
    return my_number;
}

std::string Vehicle::name()
{
    return my_name;
}

VehicleType Vehicle::vehicleType()
{
    return my_vehicleType;
}

path Vehicle::vehiclePath()
{
    return my_path;
}

uint8 Vehicle::currentState()
{
    return my_state;
}

DriverType Vehicle::driverType()
{
    return my_driverType;
}

direction Vehicle::vehicleDirection()
{
    return my_direction;
}

uint8 Vehicle::laneNumber()
{
    return my_laneNumber;
}

uint8 Vehicle::maxSpeed()
{
    return my_maxSpeed;
}