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
    if (my_state & IN_INTERSECTION)
    {
        my_timeInIntersection += simulation_params.time_step;
    }
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
*   Name: changeLane
*
*   Description: Changes the velocity vector to change lanes.
*
*   Input: direction_ -> Direction of the lane change (LEFT or RIGHT).
*
*   Output: N/A
*
*/
void Vehicle::changeLane(path direction_)
{
    if(direction_ != LEFT && direction_ != RIGHT)
    {
        SWERRINT(direction_);
        return;
    }
    
    float velocity_magnitude = MAGNITUDE(my_currentVelocity[x], my_currentVelocity[y]);

    switch(my_direction)
    {
        case(NORTH):
        {
            if(direction_ == LEFT)
            {
                my_currentVelocity[x] = -1 * velocity_magnitude * sin(-1 * PI / 6);
                my_currentVelocity[y] = velocity_magnitude * cos(-1 * PI / 6);
            }
            else
            {
                my_currentVelocity[x] = -1 * velocity_magnitude * sin(PI / 6);
                my_currentVelocity[y] = velocity_magnitude * cos(PI / 6);
            }
        }
            break;
        case(SOUTH):
        {
            if(direction_ == LEFT)
            {
                my_currentVelocity[x] = velocity_magnitude * sin(-1 * PI / 6);
                my_currentVelocity[y] = -1 * velocity_magnitude * cos(-1 * PI / 6);
            }
            else
            {
                my_currentVelocity[x] = velocity_magnitude * sin(PI / 6);
                my_currentVelocity[y] = -1 * velocity_magnitude * cos(PI / 6);
            }
        }
            break;
        case(EAST):
        {
            if(direction_ == LEFT)
            {
                my_currentVelocity[x] = velocity_magnitude * cos(-1 * PI / 6);
                my_currentVelocity[y] = -1 * velocity_magnitude * sin(-1 * PI / 6);
            }
            else
            {
                my_currentVelocity[x] = velocity_magnitude * cos(PI / 6);
                my_currentVelocity[y] = -1 * velocity_magnitude * sin(PI / 6);
            }
        }
            break;
        case(WEST):
        {
            if(direction_ == LEFT)
            {
                my_currentVelocity[x] = -1 * velocity_magnitude * cos(PI / 6);
                my_currentVelocity[y] = velocity_magnitude * sin(PI / 6);
            }
            else
            {
                my_currentVelocity[x] = -1 * velocity_magnitude * cos(-1 * PI / 6);
                my_currentVelocity[y] = velocity_magnitude * sin(-1 * PI / 6);
            }
        }
            break;
        default:SWERRINT(my_direction);
    }
}

/*
*   Name: stopLaneChange
*
*   Description: Resets the velocity vector after the lane change.
*
*   Input: N/A
*
*   Output: N/A
*
*/
void Vehicle::stopLaneChange()
{
    float velocity_magnitude = MAGNITUDE(my_currentVelocity[x], my_currentVelocity[y]);

    switch(my_direction)
    {
        case(NORTH):
        {
            my_currentVelocity[x] = velocity_magnitude * DIRECTION_VECTOR.SOUTH_VECTOR[x];
            my_currentVelocity[y] = velocity_magnitude * DIRECTION_VECTOR.SOUTH_VECTOR[y];
        }
            break;
        case(SOUTH):
        {
            my_currentVelocity[x] = velocity_magnitude * DIRECTION_VECTOR.NORTH_VECTOR[x];
            my_currentVelocity[y] = velocity_magnitude * DIRECTION_VECTOR.NORTH_VECTOR[y];
        }
            break;
        case(EAST):
        {
            my_currentVelocity[x] = velocity_magnitude * DIRECTION_VECTOR.WEST_VECTOR[x];
            my_currentVelocity[y] = velocity_magnitude * DIRECTION_VECTOR.WEST_VECTOR[y];
        }
            break;
        case(WEST):
        {
            my_currentVelocity[x] = velocity_magnitude * DIRECTION_VECTOR.EAST_VECTOR[x];
            my_currentVelocity[y] = velocity_magnitude * DIRECTION_VECTOR.EAST_VECTOR[y];
        }
            break;
        default: SWERRINT(my_direction);
    }
}

/*
*   Name: correctLane
*
*   Description: Checks if the vehicle is in the given lane.
*
*   Input: lane_ -> The lane that is being checked.
*
*   Output: True if it is a viable lane for the vehicle, false otherwise.
*
*/
bool Vehicle::correctLane(Lane* lane_)
{
    switch(my_path)
    {
        case(LEFT):
        {
            if(lane_->lanePath() == LEFT ||
               lane_->lanePath() == STRAIGHT_LEFT ||
               lane_->lanePath() == ALL_PATHS)
            {
                changeState(CORRECT_LANE, ADD);
                return true;
            }
        }
            break;
        case(STRAIGHT):
        {
            if(lane_->lanePath() == STRAIGHT ||
               lane_->lanePath() == STRAIGHT_LEFT ||
               lane_->lanePath() == STRAIGHT_RIGHT ||
               lane_->lanePath() == ALL_PATHS)
            {
                changeState(CORRECT_LANE, ADD);
                return true;
            }
        }
            break;
        case(RIGHT):
        {
            if(lane_->lanePath() == RIGHT ||
               lane_->lanePath() == STRAIGHT_RIGHT ||
               lane_->lanePath() == ALL_PATHS)
            {
                changeState(CORRECT_LANE, ADD);
                return true;
            }
        }
            break;
        default: SWERRINT(my_path);
    }
    return false;
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

/*
*   Name: setLane
*
*   Description: Changes the lane the vehicle is in.
*
*   Input: lane_number_ -> The lane number of the lane the vehicle is now in.
*
*   Output: N/A
*
*/
void Vehicle::setLane(uint8 lane_number_)
{
    my_laneNumber = lane_number_;
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

float Vehicle::timeInIntersection()
{
    return my_timeInIntersection;
}