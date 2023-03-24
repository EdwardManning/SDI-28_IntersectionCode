#include "Vehicle.h"

//Function definitions for Vehicle.h

Vehicle::Vehicle()
{
    //leave blank for initialization
}

Vehicle::Vehicle(uint32 number_, path path_, Lane* lane_)
{
    SWERRINT(number_<<3 + path_);
}

//the following should never be used
Vehicle::Vehicle(uint32 number_, path path_, Lane* lane_, DriverType driver_type_)
{
    SWERRINT(number_<<5 + path_<<2 + driver_type_);
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
    if(simulation_params.print_vehicle_info)
    { 
        printStep();
    }

    my_totalTime += simulation_params.time_step;

    if (my_state & IN_INTERSECTION)
    {
        my_timeInIntersection += simulation_params.time_step;
    }
    if (my_state & DRIVING) //if driving
    {
        my_currentPosition[x] += my_currentVelocity[x] * simulation_params.time_step;
        my_currentPosition[y] += my_currentVelocity[y] * simulation_params.time_step;
        if (MAGNITUDE(my_currentVelocity[x], my_currentVelocity[y]) == my_maxSpeed)
        {
            my_timeAtMaxSpeed += simulation_params.time_step;
        }
    }
    else
    {
        //we are stopped, this can be caused by low velocity in Simulation::run()
        //which means we may need to set the velocity to 0
        my_currentVelocity[x] = 0;
        my_currentVelocity[y] = 0;
        my_currentAcceleration[x] = 0;
        my_currentAcceleration[y] = 0;
        my_accelerationMagnitude = 0;
        my_stopTime += simulation_params.time_step;
    }
    consumeFuel();
    draw();
    if(my_totalTime > 200)
    {
        throw vehicle_time_fail();
    }
    else if((my_state & IN_INTERSECTION) && (my_state & THROUGH_INTERSECTION))
    {
        //hard swerr
        //so we know which got hit
        SWERRINT(my_state);
        throw impossible_state_fail();
    } 
    else if((my_state & CHANGING_LANES) && (my_state & CORRECT_LANE))
    {
        //hard swerr
        //so we know which got hit
        SWERRINT(my_state);
        throw impossible_state_fail();
    }
    else if((my_state & ACCELERATING) && (my_state & DECELERATING))
    {
        //hard swerr
        //so we know which got hit
        SWERRINT(my_state);
        throw impossible_state_fail();
    }
    else if ((my_state & TURNING) && !(my_state & IN_INTERSECTION))
    {
        //hard swerr
        //so we know which got hit
        SWERRINT(my_state);
        throw impossible_state_fail();
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
bool Vehicle::accelerate()
{
    bool accelerationComplete = false;

    if((my_state & ACCELERATING) && (my_state & DECELERATING))
    {
        SWERRINT(my_state);
        return true;
    }

    bool dot = my_dot;
    bool not_dot = !dot; //if dot is y then x, otherwise y
    
    float theta;

    if (my_currentVelocity[dot] != 0 || my_currentVelocity[not_dot] != 0)
    {
        updateUnitVector();
    }

    if (my_currentVelocity[dot] == 0 || my_currentVelocity[not_dot] == 0)
    {
        theta = 0;
    }
    else
    {
        theta = atan(my_currentVelocity[not_dot] / my_currentVelocity[dot]);
    }
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);

    cos_theta *= cos_theta < 0 ? -1 : 1;
    sin_theta *= sin_theta < 0 ? -1 : 1;

    my_currentAcceleration[dot] = my_accelerationMagnitude * cos_theta;
    my_currentAcceleration[not_dot] = my_accelerationMagnitude * sin_theta;
    
    bool dot_positive;
    bool not_dot_positive;

    if((my_currentVelocity[dot] == 0) && (my_currentVelocity[not_dot] == 0))
    {
        dot_positive = my_unitVector[dot] >= 0;
        not_dot_positive = my_unitVector[not_dot] >= 0;
    }
    else
    {
        dot_positive = isPositive<float>(my_currentVelocity[dot]);
        not_dot_positive = isPositive<float> (my_currentVelocity[not_dot]);
    }

    if (my_state & ACCELERATING)
    {
        if (dot_positive)
        {
            my_currentVelocity[dot] += my_currentAcceleration[dot] * simulation_params.time_step;
        }
        else
        {
            my_currentVelocity[dot] -= my_currentAcceleration[dot] * simulation_params.time_step;
        }
        if (not_dot_positive)
        {
            my_currentVelocity[not_dot] += my_currentAcceleration[not_dot] * simulation_params.time_step;
        }
        else
        {
            my_currentVelocity[not_dot] -= my_currentAcceleration[not_dot] * simulation_params.time_step;
        }
        float velocity_magnitude = MAGNITUDE(my_currentVelocity[x], my_currentVelocity[y]);
        if(my_targetSpeed == my_maxSpeed)
        {
            if(velocity_magnitude >= my_targetSpeed)
            {   
                if (dot_positive)
                {
                    my_currentVelocity[dot] =  my_maxSpeed * cos_theta;
                }
                else
                {
                    my_currentVelocity[dot] = -1 * my_maxSpeed * cos_theta;
                }
                if (not_dot_positive)
                {
                    my_currentVelocity[not_dot] =  my_maxSpeed * sin_theta;
                }
                else
                {
                    my_currentVelocity[not_dot] = -1 * my_maxSpeed * sin_theta;
                }
                my_accelerationMagnitude = 0;
                my_currentAcceleration[dot] = 0;
                my_currentAcceleration[not_dot] = 0;
                accelerationComplete = true;
            }
        }
        else
        {
            if(velocity_magnitude >= my_targetSpeed)
            {   
                if (dot_positive)
                {
                    my_currentVelocity[dot] =  my_targetSpeed * cos_theta;
                }
                else
                {
                    my_currentVelocity[dot] = -1 * my_targetSpeed * cos_theta;
                }
                if (not_dot_positive)
                {
                    my_currentVelocity[not_dot] =  my_targetSpeed * sin_theta;
                }
                else
                {
                    my_currentVelocity[not_dot] = -1 * my_targetSpeed * sin_theta;
                }
                my_accelerationMagnitude = 0;
                my_currentAcceleration[dot] = 0;
                my_currentAcceleration[not_dot] = 0;
                accelerationComplete = true;
            }
        }
        
    } 
    else if (my_state & DECELERATING)
    {
        if (dot_positive)
        {
            my_currentVelocity[dot] -= my_currentAcceleration[dot] * simulation_params.time_step;
        }
        else
        {
            my_currentVelocity[dot] += my_currentAcceleration[dot] * simulation_params.time_step;
        }
        if (not_dot_positive)
        {
            my_currentVelocity[not_dot] -= my_currentAcceleration[not_dot] * simulation_params.time_step;
        }
        else
        {
            my_currentVelocity[not_dot] += my_currentAcceleration[not_dot] * simulation_params.time_step;
        }
        float velocity_magnitude = MAGNITUDE(my_currentVelocity[x], my_currentVelocity[y]);
        if(my_targetSpeed == 0)
        {
            if((velocity_magnitude - my_accelerationMagnitude * simulation_params.time_step) <= 0.0001)
            {
                my_currentVelocity[dot] = 0;
                my_currentVelocity[not_dot] = 0;
                my_accelerationMagnitude = 0;
                my_currentAcceleration[dot] = 0;
                my_currentAcceleration[not_dot] = 0;
                accelerationComplete = true;
            }
        }
        else
        {
            if((velocity_magnitude - my_accelerationMagnitude * simulation_params.time_step) <= my_targetSpeed)
            {   
                if (dot_positive)
                {
                    my_currentVelocity[dot] =  my_targetSpeed * cos_theta;
                }
                else
                {
                    my_currentVelocity[dot] = -1 * my_targetSpeed * cos_theta;
                }
                if (not_dot_positive)
                {
                    my_currentVelocity[not_dot] =  my_targetSpeed * sin_theta;
                }
                else
                {
                    my_currentVelocity[not_dot] = -1 * my_targetSpeed * sin_theta;
                }
                my_accelerationMagnitude = 0;
                my_currentAcceleration[dot] = 0;
                my_currentAcceleration[not_dot] = 0;
                accelerationComplete = true;
            }
        } 
    }
    else
    {
        SWERRINT(my_state);
        return true;
    }
    if (accelerationComplete)
    {
        if(my_accelerationMagnitude != 0)
        {
            SWERRFLOAT(my_accelerationMagnitude);
        }
    }
    return accelerationComplete;
}

void Vehicle::accelerate(float target_speed_)
{
    bool dot = findComponent(my_currentVelocity[x], my_currentVelocity[y]);
    float velocity_magnitude = MAGNITUDE(my_currentVelocity[x], my_currentVelocity[y]);
    if(velocity_magnitude == 0 && (my_currentVelocity[x] != 0 || my_currentVelocity[y] != 0))
    {
        SWERRFLOAT(my_currentVelocity[x] != 0 ? my_currentVelocity[x] : my_currentVelocity[y]);
    }
    if(target_speed_ == STOP)
    {
        if(!(my_state & IN_INTERSECTION))
        {
            float distance_remaining = abs(my_stopline - my_exteriorPosition[FRONT_BUMPER][dot]);
            //this may need to be altered so that aggressive drivers vs standard drivers act differently
            my_accelerationMagnitude = neededAcceleration(my_currentVelocity[dot], distance_remaining); 
        }
        else
        {   
            // float distance = my_currentPosition[dot] - my_stopline;
            // if (distance < 0)
            // {
            //     distance *= -1;
            // }
            // if(distance < intersection_params.lane_width)
            // {
            //     my_accelerationMagnitude = my_driver->comfortableDeceleration();
            // }
            // else
            // {
            //     my_accelerationMagnitude = my_maxDeceleration;
            // }
            my_accelerationMagnitude = my_maxDeceleration;
            // if(my_direction == NORTH || my_direction == SOUTH)
            // {
            //     float distance = ((intersection_params.ew_number_of_exits - 1) * intersection_params.lane_width) - my_currentPosition[y];
            //     if(distance < 0)
            //     {
            //         distance *= -1;
            //     }
                
            //     my_accelerationMagnitude = neededAcceleration(my_currentVelocity[dot], distance);
            //     if(my_accelerationMagnitude > my_maxDeceleration)
            //     {
            //         my_accelerationMagnitude = my_maxDeceleration;
            //     }
            // }
            // else
            // {
            //     float distance = ((intersection_params.ns_number_of_exits - 1) * intersection_params.lane_width) - my_currentPosition[x];
            //     if(distance < 0)
            //     {
            //         distance *= -1;
            //     }
            //     my_accelerationMagnitude = neededAcceleration(my_currentVelocity[dot], distance);
            //     if(my_accelerationMagnitude > my_maxDeceleration)
            //     {
            //         my_accelerationMagnitude = my_maxDeceleration;
            //     }
            // }
        }
        if(my_accelerationMagnitude < 0)
        {
            my_accelerationMagnitude *= -1;
        }
        my_targetSpeed = STOP;
    }
    else if (target_speed_ == my_maxSpeed && velocity_magnitude < my_maxSpeed)
    {
        my_accelerationMagnitude = my_driver->comfortableAcceleration();
        my_targetSpeed = my_maxSpeed;
    }
    else if (target_speed_ == my_maxSpeed && velocity_magnitude > my_maxSpeed)
    {
        my_targetSpeed = my_maxSpeed;
        my_accelerationMagnitude = my_driver->comfortableDeceleration();
    }
    else
    {
        if(target_speed_ > velocity_magnitude)
        {
            my_targetSpeed = target_speed_;
            my_accelerationMagnitude = my_driver->comfortableDeceleration();
        }
        else
        {
            my_targetSpeed = target_speed_;
            my_accelerationMagnitude = my_driver->comfortableAcceleration();
        }
    }
}

bool Vehicle::accelerate(float target_speed_, float distance_remaining_)
{
    bool dot = maxComponent(my_currentVelocity[x], my_currentVelocity[y]);
    float velocity_magnitude = MAGNITUDE(my_currentVelocity[x], my_currentVelocity[y]);
    if(velocity_magnitude == 0 && (my_currentVelocity[x] != 0 || my_currentVelocity[y] != 0))
    {
        SWERRFLOAT(my_currentVelocity[x] != 0 ? my_currentVelocity[x] : my_currentVelocity[y]);
    }
    //if(!(my_state & IN_INTERSECTION))
    if(true)
    {
        //this may need to be altered so that aggressive drivers vs standard drivers act differently
        my_accelerationMagnitude = neededAcceleration((velocity_magnitude * 1.05), distance_remaining_);
        if(my_accelerationMagnitude < 0)
        {
            my_accelerationMagnitude *= -1;
        } 
        if(my_accelerationMagnitude < 0.01)
        {
            my_accelerationMagnitude = my_driver->comfortableDeceleration();
            my_currentAcceleration[x] = 0;
            my_currentAcceleration[y] = 0;
            return false;
        }
        if(my_accelerationMagnitude > my_maxDeceleration)
        {
            my_accelerationMagnitude = my_maxDeceleration;
        }
        my_targetSpeed = STOP;
    }
    return true;
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
                my_currentVelocity[y] = -1 * velocity_magnitude * cos( PI / 6);
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
                my_currentVelocity[x] =  -1 * velocity_magnitude * cos(PI / 6);
                my_currentVelocity[y] = velocity_magnitude * sin(PI / 6);
            }
            else
            {
                my_currentVelocity[x] = -1 * velocity_magnitude * cos(PI / 6);
                my_currentVelocity[y] = velocity_magnitude * sin(-1 * PI / 6);
            }
        }
            break;
        case(WEST):
        {
            if(direction_ == LEFT)
            {
                my_currentVelocity[x] = velocity_magnitude * cos(-1 * PI / 6);
                my_currentVelocity[y] = velocity_magnitude * sin(-1 * PI / 6);
            }
            else
            {
                my_currentVelocity[x] = velocity_magnitude * cos(PI / 6);
                my_currentVelocity[y] = velocity_magnitude * sin(PI / 6);
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
bool Vehicle::correctLane(Lane* lane_, bool initialization_)
{
    switch(my_path)
    {
        case(LEFT):
        {
            if(lane_->lanePath() == LEFT ||
               lane_->lanePath() == STRAIGHT_LEFT ||
               lane_->lanePath() == ALL_PATHS)
            {
                if(initialization_)
                {
                    changeState(CORRECT_LANE, ADD);
                }
                my_stoplineCenter = my_currentPosition[minComponent<float>(my_currentVelocity[x], my_currentVelocity[y])];
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
                if(initialization_)
                {
                    changeState(CORRECT_LANE, ADD);
                }
                my_stoplineCenter = my_currentPosition[minComponent<float>(my_currentVelocity[x], my_currentVelocity[y])];
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
                if(initialization_)
                {
                    changeState(CORRECT_LANE, ADD);
                }
                my_stoplineCenter = my_currentPosition[minComponent<float>(my_currentVelocity[x], my_currentVelocity[y])];
                return true;
            }
        }
            break;
        default: SWERRINT(my_path);
    }
    return false;
}

/*
*   Name: turn
*
*   Description: Turns the vehicle by changing the velocity vector before drive().
*
*   Input: N/A
*
*   Output: N/A
*
*/
void Vehicle::turn()
{
    if(my_path != LEFT && my_path != RIGHT)
    {
        SWERRINT(my_number<<11 + my_laneNumber<<3 + my_path);
    }

    bool direction_of_travel = my_direction == NORTH || my_direction == SOUTH ? y : x;
    bool direction_changed_to = direction_of_travel ? x : y; //if direction of travel is north or south then x, otherwize y

    float dot_distance_travelled = abs(my_currentPosition[direction_of_travel] - my_stopline);
    float dct_distance_travelled = abs(my_currentPosition[direction_changed_to] - my_stoplineCenter);

    float velocity_magnitude = MAGNITUDE(my_currentVelocity[x], my_currentVelocity[y]);

    float theta;

    if (my_turnRadius[direction_of_travel] > my_turnRadius[direction_changed_to])
    {
        if (my_turnRadius[direction_of_travel] - dot_distance_travelled <= my_turnRadius[direction_changed_to])
        {
            theta = (0.5 * PI) * ((dot_distance_travelled - (my_turnRadius[direction_of_travel] - my_turnRadius[direction_changed_to])) / my_turnRadius[direction_changed_to]);
        }
        else
        {
            theta = 0;
        }
    }
    else
    {
        if ((my_currentPosition[direction_of_travel] - my_stopline) >= my_turnRadius[direction_of_travel])
        {
            theta = (PI / 2);
        }
        else
        {
            theta = (0.5 * PI) * ((my_currentPosition[direction_of_travel] - my_stopline) / my_turnRadius[direction_of_travel]);
        }
    }

    my_currentVelocity[direction_of_travel] = velocity_magnitude * my_modifier[direction_of_travel](theta);
    my_currentVelocity[direction_changed_to] = velocity_magnitude * my_modifier[direction_changed_to](theta);
    
}

/*
*   Name: stopTurn
*
*   Description: Sets the unit vector and max speed to those of the exit lane.
*
*   Input: lane_ -> The exit lane in question.
*
*   Output: N/A
*
*/
void Vehicle::stopTurn(Lane* lane_)
{
    float velocity_magnitude = MAGNITUDE(my_currentVelocity[x], my_currentVelocity[y]);

    my_maxSpeed = my_driver->modifier() * lane_->speedLimit();
    my_dot = maxComponent<int8>(lane_->unitVector()[x], lane_->unitVector()[y]);

    my_currentVelocity[x] = velocity_magnitude * lane_->unitVector()[x];
    my_currentVelocity[y] = velocity_magnitude * lane_->unitVector()[y];
    
    updateUnitVector();
}

bool Vehicle::collisionCheck(Vehicle* vehicle_)
{
    //dot means direction of travel
    bool dot = maxComponent<float>(my_currentVelocity[x], my_currentVelocity[y]);
    bool not_dot = !dot; //y is true so if dot is y then x, otherwise y

    for(uint8 i = 0; i < TOTAL_POINTS; i++)
    {
        float dot_position = vehicle_->exteriorPosition(getVehiclePoint(i))[dot];
        float not_dot_position = vehicle_->exteriorPosition(getVehiclePoint(i))[not_dot];

        float dot_max = max<float>(my_exteriorPosition[FRONT_BUMPER][dot], my_exteriorPosition[BACK_BUMPER][dot]);
        float dot_min = min<float>(my_exteriorPosition[FRONT_BUMPER][dot], my_exteriorPosition[BACK_BUMPER][dot]);
        float not_dot_max = max<float>(my_exteriorPosition[FRONT_LEFT][not_dot], my_exteriorPosition[FRONT_RIGHT][not_dot]);
        float not_dot_min = min<float>(my_exteriorPosition[FRONT_LEFT][not_dot], my_exteriorPosition[FRONT_RIGHT][not_dot]);

        if ((not_dot_position > not_dot_min)
            && (not_dot_position < not_dot_max)
            && (dot_position > dot_min)
            && (dot_position < dot_max))
        {
            return true;
        }
    }
    return false;
}

bool Vehicle::lightChange(lightColour colour_)
{
    switch(colour_)
    {
        case(GREEN):
        {
            return false;
        }
            break;
        case(YELLOW):
        {
            if(yellowLightAnalysis())
            {
                return true;
            }
            else
            {
                my_runningLight = true;
                return false;
            }
        }
            break;
        case(RED):
        {
            if(redLightAnalysis())
            {
                return true;
            }
            else
            {
                my_runningLight = true;
                return false;
            }
        }
            break;
        default: SWERRINT(colour_);
    }
    return false;
}

bool Vehicle::yellowLightAnalysis()
{
    //only gets called before intersection
    //true causes slow down, false keeps going
    return abs(stoppingDistance<float>(my_currentVelocity[my_dot], my_driver->comfortableDeceleration())) < abs(my_stopline - my_exteriorPosition[FRONT_BUMPER][my_dot]);
}

bool Vehicle::yellowLightAnalysis(float current_acceleration_)
{
    //used if vehicle already slowing down due to proximity to other vehicle, or due to lane change
    return ((abs(stoppingDistance<float>(my_currentVelocity[my_dot], my_driver->comfortableDeceleration())) < abs(my_stopline - my_exteriorPosition[FRONT_BUMPER][my_dot])) &&
            (my_driver->comfortableAcceleration() > current_acceleration_));
}

bool Vehicle::redLightAnalysis()
{
    //only gets called before intersection
    //true causes slow down, false keeps going
    return abs(stoppingDistance<float>(my_currentVelocity[my_dot], my_maxDeceleration)) < abs(my_stopline - my_exteriorPosition[FRONT_BUMPER][my_dot]);
}

bool Vehicle::redLightAnalysis(float current_acceleration_)
{
    //used if vehicle already slowing down due to proximity to other vehicle, or due to lane change
    //second part of the statement should always be true but it is being tested anyways
    return (abs(stoppingDistance<float>(my_currentVelocity[my_dot], my_maxDeceleration)) < abs(my_stopline - my_exteriorPosition[FRONT_BUMPER][my_dot]) &&
            (my_maxDeceleration > current_acceleration_));
}

/*
*   Name: changeState
*
*   Description: Adds or removes a state from the vehicle's my_state.
*
*   Input: state_  -> The state being added or removed.
*          adding_ -> Boolean stating weather the state is being added or removed.
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

/*
*   Name: completed
*
*   Description: Sets my_completion status to true (meaning it has 
*                completed the intersection).
*
*   Input: N/A
*
*   Output: N/A
*
*/
void Vehicle::completed()
{
    my_completionStatus = true;

    if(simulation_params.print_vehicle_info)
    { 
        printStep();
        printFinalInformation();
    }
}


/*
*   Name: draw
*
*   Description: Determines the new exterior points. Once
*                the center of the vehicle moves, so do the exterior
*                points. This function defines the points based on
*                the current center position and velocity vector.
*
*   Input: N/A
*
*   Output: N/A
*
*/
// void Vehicle::draw(bool initialization_)
// {
//     //dot means direction of travel
//     if (MAGNITUDE(my_currentVelocity[x], my_currentVelocity[y]) == 0 ||
//         !(my_state & DRIVING))
//     {
//         return; //if you are stopped there is no change to your position
//     }
//     bool dot = findComponent(my_exteriorPosition[FRONT_BUMPER], my_exteriorPosition[BACK_BUMPER]);
//     bool not_dot = !dot; //if the direction of travel is true (y) then x is not 
//     bool positive_dot = isPositive<float>(my_currentVelocity[dot]);
    

//     if((my_state & TURNING) && (my_currentVelocity[not_dot] != 0))
//     {
//         float theta = atan(my_currentVelocity[not_dot] / my_currentVelocity[dot]);

//         if(positive_dot)
//         {
//             my_exteriorPosition[FRONT_BUMPER][dot] = my_currentPosition[dot] + (0.5 * vehicle_params.vehicle_length * cos(theta));
//             my_exteriorPosition[FRONT_BUMPER][not_dot] = my_currentPosition[not_dot] + (0.5 * vehicle_params.vehicle_length * sin(theta));
        
//             my_exteriorPosition[BACK_BUMPER][dot] = my_currentPosition[dot] - (0.5 * vehicle_params.vehicle_length * cos(theta));
//             my_exteriorPosition[BACK_BUMPER][not_dot] = my_currentPosition[not_dot] - (0.5 * vehicle_params.vehicle_length * sin(theta));

//             if(dot) //headed south
//             {
//                 my_exteriorPosition[FRONT_LEFT][dot] = my_exteriorPosition[FRONT_BUMPER][dot] + (0.5 * vehicle_params.vehicle_width * sin(theta));
//                 my_exteriorPosition[FRONT_LEFT][not_dot] = my_exteriorPosition[FRONT_BUMPER][not_dot] - (0.5 * vehicle_params.vehicle_width * cos(theta));

//                 my_exteriorPosition[FRONT_RIGHT][dot] = my_exteriorPosition[FRONT_BUMPER][dot] - (0.5 * vehicle_params.vehicle_width * sin(theta));
//                 my_exteriorPosition[FRONT_RIGHT][not_dot] = my_exteriorPosition[FRONT_BUMPER][not_dot] + (0.5 * vehicle_params.vehicle_width * cos(theta));

//                 my_exteriorPosition[BACK_LEFT][dot] = my_exteriorPosition[BACK_BUMPER][dot] + (0.5 * vehicle_params.vehicle_width * sin(theta));
//                 my_exteriorPosition[BACK_LEFT][not_dot] = my_exteriorPosition[BACK_BUMPER][not_dot] - (0.5 * vehicle_params.vehicle_width * cos(theta));

//                 my_exteriorPosition[BACK_RIGHT][dot] = my_exteriorPosition[BACK_BUMPER][dot] - (0.5 * vehicle_params.vehicle_width * sin(theta));
//                 my_exteriorPosition[BACK_RIGHT][not_dot] = my_exteriorPosition[BACK_BUMPER][not_dot] + (0.5 * vehicle_params.vehicle_width * cos(theta));
//             }
//             else //headed east
//             {
//                 my_exteriorPosition[FRONT_LEFT][dot] = my_exteriorPosition[FRONT_BUMPER][dot] - (0.5 * vehicle_params.vehicle_width * sin(theta));
//                 my_exteriorPosition[FRONT_LEFT][not_dot] = my_exteriorPosition[FRONT_BUMPER][not_dot] + (0.5 * vehicle_params.vehicle_width * cos(theta));

//                 my_exteriorPosition[FRONT_RIGHT][dot] = my_exteriorPosition[FRONT_BUMPER][dot] + (0.5 * vehicle_params.vehicle_width * sin(theta));
//                 my_exteriorPosition[FRONT_RIGHT][not_dot] = my_exteriorPosition[FRONT_BUMPER][not_dot] - (0.5 * vehicle_params.vehicle_width * cos(theta));

//                 my_exteriorPosition[BACK_LEFT][dot] = my_exteriorPosition[BACK_BUMPER][dot] - (0.5 * vehicle_params.vehicle_width * sin(theta));
//                 my_exteriorPosition[BACK_LEFT][not_dot] = my_exteriorPosition[BACK_BUMPER][not_dot] + (0.5 * vehicle_params.vehicle_width * cos(theta));

//                 my_exteriorPosition[BACK_RIGHT][dot] = my_exteriorPosition[BACK_BUMPER][dot] + (0.5 * vehicle_params.vehicle_width * sin(theta));
//                 my_exteriorPosition[BACK_RIGHT][not_dot] = my_exteriorPosition[BACK_BUMPER][not_dot] - (0.5 * vehicle_params.vehicle_width * cos(theta));
//             }
            
//         }
//         else
//         {
//             my_exteriorPosition[FRONT_BUMPER][dot] = my_currentPosition[dot] - (0.5 * vehicle_params.vehicle_length * cos(theta));
//             my_exteriorPosition[FRONT_BUMPER][not_dot] = my_currentPosition[not_dot] - (0.5 * vehicle_params.vehicle_length * sin(theta));
        
//             my_exteriorPosition[BACK_BUMPER][dot] = my_currentPosition[dot] + (0.5 * vehicle_params.vehicle_length * cos(theta));
//             my_exteriorPosition[BACK_BUMPER][not_dot] = my_currentPosition[not_dot] + (0.5 * vehicle_params.vehicle_length * sin(theta));

//             if(dot) //headed north
//             {
//                 my_exteriorPosition[FRONT_LEFT][dot] = my_exteriorPosition[FRONT_BUMPER][dot] - (0.5 * vehicle_params.vehicle_width * sin(theta));
//                 my_exteriorPosition[FRONT_LEFT][not_dot] = my_exteriorPosition[FRONT_BUMPER][not_dot] + (0.5 * vehicle_params.vehicle_width * cos(theta));

//                 my_exteriorPosition[FRONT_RIGHT][dot] = my_exteriorPosition[FRONT_BUMPER][dot] + (0.5 * vehicle_params.vehicle_width * sin(theta));
//                 my_exteriorPosition[FRONT_RIGHT][not_dot] = my_exteriorPosition[FRONT_BUMPER][not_dot] - (0.5 * vehicle_params.vehicle_width * cos(theta));

//                 my_exteriorPosition[BACK_LEFT][dot] = my_exteriorPosition[BACK_BUMPER][dot] - (0.5 * vehicle_params.vehicle_width * sin(theta));
//                 my_exteriorPosition[BACK_LEFT][not_dot] = my_exteriorPosition[BACK_BUMPER][not_dot] + (0.5 * vehicle_params.vehicle_width * cos(theta));

//                 my_exteriorPosition[BACK_RIGHT][dot] = my_exteriorPosition[BACK_BUMPER][dot] + (0.5 * vehicle_params.vehicle_width * sin(theta));
//                 my_exteriorPosition[BACK_RIGHT][not_dot] = my_exteriorPosition[BACK_BUMPER][not_dot] - (0.5 * vehicle_params.vehicle_width * cos(theta));
//             }
//             else //headed west
//             {
//                 my_exteriorPosition[FRONT_LEFT][dot] = my_exteriorPosition[FRONT_BUMPER][dot] + (0.5 * vehicle_params.vehicle_width * sin(theta));
//                 my_exteriorPosition[FRONT_LEFT][not_dot] = my_exteriorPosition[FRONT_BUMPER][not_dot] - (0.5 * vehicle_params.vehicle_width * cos(theta));

//                 my_exteriorPosition[FRONT_RIGHT][dot] = my_exteriorPosition[FRONT_BUMPER][dot] - (0.5 * vehicle_params.vehicle_width * sin(theta));
//                 my_exteriorPosition[FRONT_RIGHT][not_dot] = my_exteriorPosition[FRONT_BUMPER][not_dot] + (0.5 * vehicle_params.vehicle_width * cos(theta));

//                 my_exteriorPosition[BACK_LEFT][dot] = my_exteriorPosition[BACK_BUMPER][dot] + (0.5 * vehicle_params.vehicle_width * sin(theta));
//                 my_exteriorPosition[BACK_LEFT][not_dot] = my_exteriorPosition[BACK_BUMPER][not_dot] - (0.5 * vehicle_params.vehicle_width * cos(theta));

//                 my_exteriorPosition[BACK_RIGHT][dot] = my_exteriorPosition[BACK_BUMPER][dot] - (0.5 * vehicle_params.vehicle_width * sin(theta));
//                 my_exteriorPosition[BACK_RIGHT][not_dot] = my_exteriorPosition[BACK_BUMPER][not_dot] + (0.5 * vehicle_params.vehicle_width * cos(theta));
//             }
//         }
//     }
//     else
//     {
//         if(positive_dot)
//         {
//             my_exteriorPosition[FRONT_BUMPER][dot] = my_currentPosition[dot] - (0.5 * vehicle_params.vehicle_length);
//             my_exteriorPosition[BACK_BUMPER][dot] = my_currentPosition[dot] + (0.5 * vehicle_params.vehicle_length);
//         }
//         else
//         {
//             my_exteriorPosition[FRONT_BUMPER][dot] = my_currentPosition[dot] - (0.5 * vehicle_params.vehicle_length);
//             my_exteriorPosition[BACK_BUMPER][dot] = my_currentPosition[dot] + (0.5 * vehicle_params.vehicle_length);
//         }
        

//         if((my_exteriorPosition[FRONT_BUMPER][not_dot] != my_exteriorPosition[BACK_BUMPER][not_dot]) 
//            || (my_exteriorPosition[FRONT_BUMPER][not_dot] != my_currentPosition[not_dot])
//            || initialization_)
//         {
//             //if a turn/lanechange just ended then this could be a possible situation
//             my_exteriorPosition[FRONT_BUMPER][not_dot] = my_currentPosition[not_dot];
//             my_exteriorPosition[BACK_BUMPER][not_dot] = my_currentPosition[not_dot];

//             if(positive_dot)
//             {
//                 if(dot) //headed south
//                 {
//                     my_exteriorPosition[FRONT_LEFT][not_dot] = my_currentPosition[not_dot] + (0.5 * vehicle_params.vehicle_width); 
//                     my_exteriorPosition[BACK_LEFT][not_dot] = my_currentPosition[not_dot] + (0.5 * vehicle_params.vehicle_width);

//                     my_exteriorPosition[FRONT_RIGHT][not_dot] = my_currentPosition[not_dot] - (0.5 * vehicle_params.vehicle_width);
//                     my_exteriorPosition[BACK_RIGHT][not_dot] = my_currentPosition[not_dot] - (0.5 * vehicle_params.vehicle_width);
//                 }
//                 else //headed east
//                 {
//                     my_exteriorPosition[FRONT_LEFT][not_dot] = my_currentPosition[not_dot] - (0.5 * vehicle_params.vehicle_width); 
//                     my_exteriorPosition[BACK_LEFT][not_dot] = my_currentPosition[not_dot] - (0.5 * vehicle_params.vehicle_width);

//                     my_exteriorPosition[FRONT_RIGHT][not_dot] = my_currentPosition[not_dot] + (0.5 * vehicle_params.vehicle_width);
//                     my_exteriorPosition[BACK_RIGHT][not_dot] = my_currentPosition[not_dot] + (0.5 * vehicle_params.vehicle_width);
//                 }
//             }
//             else
//             {
//                 if(dot) //headed north
//                 {
//                     my_exteriorPosition[FRONT_LEFT][not_dot] = my_currentPosition[not_dot] - (0.5 * vehicle_params.vehicle_width); 
//                     my_exteriorPosition[BACK_LEFT][not_dot] = my_currentPosition[not_dot] - (0.5 * vehicle_params.vehicle_width);

//                     my_exteriorPosition[FRONT_RIGHT][not_dot] = my_currentPosition[not_dot] + (0.5 * vehicle_params.vehicle_width);
//                     my_exteriorPosition[BACK_RIGHT][not_dot] = my_currentPosition[not_dot] + (0.5 * vehicle_params.vehicle_width);
//                 }
//                 else //headed west
//                 {
//                     my_exteriorPosition[FRONT_LEFT][not_dot] = my_currentPosition[not_dot] + (0.5 * vehicle_params.vehicle_width); 
//                     my_exteriorPosition[BACK_LEFT][not_dot] = my_currentPosition[not_dot] + (0.5 * vehicle_params.vehicle_width);

//                     my_exteriorPosition[FRONT_RIGHT][not_dot] = my_currentPosition[not_dot] - (0.5 * vehicle_params.vehicle_width);
//                     my_exteriorPosition[BACK_RIGHT][not_dot] = my_currentPosition[not_dot] - (0.5 * vehicle_params.vehicle_width);
//                 }
//             }
//         }

//         my_exteriorPosition[FRONT_LEFT][dot] = my_exteriorPosition[FRONT_BUMPER][dot];
//         my_exteriorPosition[FRONT_RIGHT][dot] = my_exteriorPosition[FRONT_BUMPER][dot];

//         my_exteriorPosition[BACK_LEFT][dot] = my_exteriorPosition[BACK_BUMPER][dot];
//         my_exteriorPosition[BACK_RIGHT][dot] = my_exteriorPosition[BACK_BUMPER][dot];
//     }
// }

void Vehicle::draw(bool initialization_)
{
    if (MAGNITUDE(my_currentVelocity[x], my_currentVelocity[y]) == 0 ||
        !(my_state & DRIVING))
    {
        return; //if you are stopped there is no change to your position
    }
    bool dot;

    if(initialization_)
    {
        dot = findComponent(my_currentVelocity[x], my_currentVelocity[y]);
    }
    else
    {
        dot = findComponent(my_exteriorPosition[FRONT_BUMPER], my_exteriorPosition[BACK_BUMPER]);
    }

    bool not_dot = !dot;

    bool dot_is_positive = my_currentVelocity[dot] >= 0;
    bool not_dot_is_positive = my_currentVelocity[not_dot] >= 0;

    // bool dot_is_positive = my_exteriorPosition[FRONT_BUMPER][dot] > my_exteriorPosition[BACK_BUMPER][dot];
    // bool not_dot_positive = my_currentVelocity[not_dot] >= 0;

    int8 dot_modifier = dot_is_positive ? 1 : -1;
    int8 not_dot_modifier = not_dot_is_positive ? 1 : -1;

    float dot_velocity_magnitude = dot_modifier * my_currentVelocity[dot];
    float not_dot_velocity_magnitude = not_dot_modifier * my_currentVelocity[not_dot];

    float theta;

    uint8 half_length = vehicle_params.vehicle_length / 2;
    uint8 half_width = vehicle_params.vehicle_width / 2;

    if(not_dot_velocity_magnitude == 0 || (my_state & CHANGING_LANES))
    {
        theta = 0;
    }
    else if(dot_velocity_magnitude != 0)
    {
        theta = atan(not_dot_velocity_magnitude / dot_velocity_magnitude);
    }
    else
    {
        SWERRFLOAT(my_currentVelocity[dot]);
        theta = 0;
    }

    my_exteriorPosition[FRONT_BUMPER][dot] = my_currentPosition[dot] + (dot_modifier * half_length * cos(theta));
    my_exteriorPosition[FRONT_BUMPER][not_dot] = my_currentPosition[not_dot] + (not_dot_modifier * half_length * sin(theta));

    my_exteriorPosition[BACK_BUMPER][dot] = my_currentPosition[dot] - (dot_modifier * half_length * cos(theta));
    my_exteriorPosition[BACK_BUMPER][not_dot] = my_currentPosition[not_dot] - (not_dot_modifier * half_length * sin(theta));

    if(dot)
    {
        //if we are travelling along the y axis then the left will be in the same direction as you are pointed
        //so if we are headed north we are travelling with a negative velocity which means that the [x] left position
        //will be less than the [x] center position
        my_exteriorPosition[FRONT_LEFT][dot] = my_exteriorPosition[FRONT_BUMPER][dot] - (not_dot_modifier * half_width * sin(theta));
        my_exteriorPosition[FRONT_LEFT][not_dot] = my_exteriorPosition[FRONT_BUMPER][not_dot] + (dot_modifier * half_width * cos(theta));

        my_exteriorPosition[FRONT_RIGHT][dot] = my_exteriorPosition[FRONT_BUMPER][dot] + (not_dot_modifier * half_width * sin(theta));
        my_exteriorPosition[FRONT_RIGHT][not_dot] = my_exteriorPosition[FRONT_BUMPER][not_dot] - (dot_modifier * half_width * cos(theta));

        my_exteriorPosition[BACK_LEFT][dot] = my_exteriorPosition[BACK_BUMPER][dot] - (not_dot_modifier * half_width * sin(theta));
        my_exteriorPosition[BACK_LEFT][not_dot] = my_exteriorPosition[BACK_BUMPER][not_dot] + (dot_modifier * half_width * cos(theta));

        my_exteriorPosition[BACK_RIGHT][dot] = my_exteriorPosition[BACK_BUMPER][dot] + (not_dot_modifier * half_width * sin(theta));
        my_exteriorPosition[BACK_RIGHT][not_dot] = my_exteriorPosition[BACK_BUMPER][not_dot] - (dot_modifier * half_width * cos(theta));
    }
    else
    {
        //if we are travelling along the x axis than the opposite of what was true above is true now
        my_exteriorPosition[FRONT_LEFT][dot] = my_exteriorPosition[FRONT_BUMPER][dot] + (not_dot_modifier * half_width * sin(theta));
        my_exteriorPosition[FRONT_LEFT][not_dot] = my_exteriorPosition[FRONT_BUMPER][not_dot] - (dot_modifier * half_width * cos(theta));

        my_exteriorPosition[FRONT_RIGHT][dot] = my_exteriorPosition[FRONT_BUMPER][dot] - (not_dot_modifier * half_width * sin(theta));
        my_exteriorPosition[FRONT_RIGHT][not_dot] = my_exteriorPosition[FRONT_BUMPER][not_dot] + (dot_modifier * half_width * cos(theta));

        my_exteriorPosition[BACK_LEFT][dot] = my_exteriorPosition[BACK_BUMPER][dot] + (not_dot_modifier * half_width * sin(theta));
        my_exteriorPosition[BACK_LEFT][not_dot] = my_exteriorPosition[BACK_BUMPER][not_dot] - (dot_modifier * half_width * cos(theta));

        my_exteriorPosition[BACK_RIGHT][dot] = my_exteriorPosition[BACK_BUMPER][dot] - (not_dot_modifier * half_width * sin(theta));
        my_exteriorPosition[BACK_RIGHT][not_dot] = my_exteriorPosition[BACK_BUMPER][not_dot] + (dot_modifier * half_width * cos(theta));
    }
}

void Vehicle::updateUnitVector()
{
    if (my_currentVelocity[x] > 0)
    {
        my_unitVector[x] = 1;
    }
    else if (my_currentVelocity[x] < 0)
    {
        my_unitVector[x] = -1;
    }
    else
    {
        my_unitVector[x] = 0;
    }

    if (my_currentVelocity[y] > 0)
    {
        my_unitVector[y] = 1;
    }
    else if (my_currentVelocity[y] < 0)
    {
        my_unitVector[y] = -1;
    }
    else
    {
        my_unitVector[y] = 0;
    }
}

bool Vehicle::checkImportantPosition(Vehicle* vehicle_)
{
    if(my_currentVelocity[my_dot] > 0)
    {
        return (vehicle_->exteriorPosition(FRONT_BUMPER)[my_dot] >= my_exteriorPosition[BACK_BUMPER][my_dot] &&
                vehicle_->exteriorPosition(FRONT_BUMPER)[my_dot] <= my_exteriorPosition[FRONT_BUMPER][my_dot]) ||
                (vehicle_->exteriorPosition(BACK_BUMPER)[my_dot] >= my_exteriorPosition[BACK_BUMPER][my_dot] &&
                vehicle_->exteriorPosition(BACK_BUMPER)[my_dot] <= (my_exteriorPosition[FRONT_BUMPER][my_dot] + (2 * my_driver->minimumFollowingDistance())));
    }
    else
    {
        return (vehicle_->exteriorPosition(FRONT_BUMPER)[my_dot] <= my_exteriorPosition[BACK_BUMPER][my_dot] &&
                vehicle_->exteriorPosition(FRONT_BUMPER)[my_dot] >= my_exteriorPosition[FRONT_BUMPER][my_dot]) ||
                (vehicle_->exteriorPosition(BACK_BUMPER)[my_dot] <= my_exteriorPosition[BACK_BUMPER][my_dot] &&
                vehicle_->exteriorPosition(BACK_BUMPER)[my_dot] >= (my_exteriorPosition[FRONT_BUMPER][my_dot] - (2 * my_driver->minimumFollowingDistance())));
    }
}

void Vehicle::requestAccelerationAdjustment(float adjustment_, bool is_positive_)
{
    if(is_positive_)
    {
        if(my_state & ACCELERATING)
        {
            if(my_accelerationMagnitude + adjustment_ > (my_driver->comfortableAcceleration() * 2))
            {
                adjustAccelerationMagnitude(my_driver->comfortableAcceleration() * 2);
            }
            else
            {
                adjustAccelerationMagnitude(adjustment_, is_positive_);
            }
        }
        else
        {
            if(my_accelerationMagnitude + adjustment_ > (my_maxDeceleration))
            {
                adjustAccelerationMagnitude(my_maxDeceleration);
            }
            else
            {
                adjustAccelerationMagnitude(adjustment_, is_positive_);
            }
        }
    }
    else
    {
        if(my_accelerationMagnitude - adjustment_ <= 0)
        {
            return;
        }
        else
        {
            adjustAccelerationMagnitude(adjustment_, is_positive_);
        }
    }
}

void Vehicle::adjustAccelerationMagnitude(float adjustment_, bool is_positive_)
{
    if(is_positive_)
    {
        my_accelerationMagnitude += adjustment_;
    }
    else
    {
        my_accelerationMagnitude -= adjustment_;
    }
}

void Vehicle::adjustAccelerationMagnitude(float acceleration_magnitude_)
{
    my_accelerationMagnitude = acceleration_magnitude_;
}

void Vehicle::setMaxSpeed(float new_max_speed_)
{
    my_maxSpeed = new_max_speed_;
}

void Vehicle::setCurrentSeparation(float separation_)
{
    my_currentSeparation = separation_;
}

void Vehicle::toggleBlinker(path direction_, bool on_)
{
    if(direction_ == LEFT)
    {
        if(my_blinker[0] != on_)
        {
            my_blinker[0] = on_;
        }
        else
        {
            SWERRINT(my_blinker[0]);
        }
    }
    else if(direction_ == RIGHT)
    {
        if(my_blinker[1] != on_)
        {
            my_blinker[1] = on_;
        }
        else
        {
            SWERRINT(my_blinker[1]);
        }
    }
    else
    {
        SWERRINT(direction_);
    }
}

void Vehicle::toggleBrakeLights(bool on_)
{
    if(my_brakeLights != on_)
    {
        my_brakeLights = on_;
    }
    else
    {
        SWERRINT(my_brakeLights);
    }
}

float Vehicle::distanceToStopComfortably()
{
    return neededDistance(MAGNITUDE(my_currentVelocity[x], my_currentVelocity[y]), my_driver->comfortableDeceleration());
}

void Vehicle::consumeFuel()
{
    if(my_vehicleType == SELF_DRIVING_CAR)
    {
        SWERRINT(my_vehicleType);
    }
    float velocity_magnitude = MAGNITUDE(my_currentVelocity[x], my_currentVelocity[y]);
    if(velocity_magnitude == 0 && (my_currentVelocity[x] != 0 || my_currentVelocity[y] != 0))
    {
        SWERRFLOAT(my_currentVelocity[x] != 0 ? my_currentVelocity[x] : my_currentVelocity[y]);
    }
    float gas_to_kgCO2 = 2.3 / 1000; //kg/ml
    float fuel_used_at_step = ((my_fuelConsumptionAtVelocity * velocity_magnitude) + my_idleFuelConsumption) * simulation_params.time_step;
    my_totalFuelConsumption += fuel_used_at_step;
    my_totalEmissions += gas_to_kgCO2 * fuel_used_at_step;
}

//Printing funtions, no need for explanation
//They print things
void Vehicle::printStartingInformation()
{
    info << "****************************************" << std::endl;
    info << "*" << std::endl;
    info << "* Vehicle Name: " << my_name << std::endl;
    info << "* Vehicle Number: " << my_number << std::endl;
    info << "* Driver Type: " << my_driver->name() << std::endl;
    info << "*" << std::endl;
    info << "* Starting Lane: " << (int)my_laneNumber << std::endl;
    info << "* Starting Road: " << DIRECTION_STR[my_direction] << std::endl;
    info << "* Path: " << PATH_STR[my_path] << std::endl;
    info << "*" << std::endl;
    info << "****************************************" << std::endl;
}

void Vehicle::printStep()
{
    info << (int)my_state << "\t" << my_currentPosition[x] << "\t" << my_currentPosition[y] << "\t";
    if(vehicle_params.print_velocieties)
    {
        info << "\t" << my_currentVelocity[x] << "\t" << my_currentVelocity[y] << "\t";
    }
    if(vehicle_params.print_accelerations)
    {
        info << "\t" << my_accelerationMagnitude<< "\t" << my_currentAcceleration[x] << "\t" << my_currentAcceleration[y] << "\t";
    }
    if(vehicle_params.print_exterior_coordinates)
    {
        for(uint8 i = 0; i < TOTAL_POINTS; i++)
        {
            info << "\t" << my_exteriorPosition[i][x] << "\t" << my_exteriorPosition[i][y] << "\t";
        }
    }
    info << std::endl;
}

void Vehicle::printFinalInformation()
{
    info << std::endl;
    info << "*******************************************************" << std::endl;
    info << my_totalTime << "\t" << my_timeInIntersection << "\t" << my_timeAtMaxSpeed << "\t" << my_stopTime << "\t" << my_totalFuelConsumption << std::endl;
    info << "*******************************************************" << std::endl;
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

uint32 Vehicle::number()
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
    return my_driver->driverType();
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

float Vehicle::totalTime()
{
    return my_totalTime;
}

float Vehicle::timeStopped()
{
    return my_stopTime;
}

float Vehicle::timeAtMaxSpeed()
{
    return my_timeAtMaxSpeed;
}

float* Vehicle::exteriorPosition(vehiclePoints vehicle_point_)
{
    return my_exteriorPosition[vehicle_point_];
}

float* Vehicle::exteriorPosition(uint8 vehicle_point_)
{
    return my_exteriorPosition[vehicle_point_];
}

bool Vehicle::isCompleted()
{
    return my_completionStatus;
}

float Vehicle::currentAccelerationMagnitude()
{
    return my_accelerationMagnitude;
}

int8* Vehicle::unitVector()
{
    return my_unitVector;
}

bool Vehicle::blinker(path direction_)
{
    if (direction_ == LEFT)
    {
        return my_blinker[0];
    }
    else if(direction_ == RIGHT)
    {
        return my_blinker[1];
    }
    else
    {
        SWERRINT(direction_);
    }
    return my_blinker[0];
}

bool Vehicle::blinker(bool direction_)
{
    return my_blinker[direction_];
}

bool Vehicle::brakeLights()
{
    return my_brakeLights;
}

float Vehicle::mimumumStoppingDistance()
{
    return my_driver->minimumStoppingDistance();
}

float Vehicle::minimumFollowingDistance()
{
    return my_driver->minimumFollowingDistance();
}

float Vehicle::slowingDistance()
{
    return my_driver->slowingDistance();
}

float Vehicle::currentSeparation()
{
    return my_currentSeparation;
}

float Vehicle::stopLine()
{
    return my_stopline;
}

bool Vehicle::goingThroughLight()
{
    return my_runningLight;
}

float Vehicle::fuelConsumed()
{
    return my_totalFuelConsumption;
}

float Vehicle::emissions()
{
    return my_totalEmissions;
}