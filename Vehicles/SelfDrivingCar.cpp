#include "./SelfDrivingCar.h"
#include "SelfDrivingCar.h"

SelfDrivingCar::SelfDrivingCar(uint32 number_, path path_, Lane* lane_)
{
    my_number = number_;
    my_vehicleType = SELF_DRIVING_CAR;
    my_driver = new SelfDriver();
    my_name = VEHICLE_TYPE_STR[my_vehicleType] + " " + std::to_string(my_number);
    my_maxSpeed = lane_->speedLimit();
    my_timeInIntersection = 0;
    my_totalTime = 0;
    my_stopTime = 0;
    my_timeAtMaxSpeed = 0;
    my_completionStatus = false;
    my_maxDeceleration = 7;
    my_accelerationMagnitude = 0;
    my_currentSeparation = -1;
    my_ignoreStatus = false;

    my_nextCommand.command_type = NULL_COMMAND;
    my_nextCommand.value = 0;
    my_nextCommand.vehicle_number = my_number;

    my_brakeLights = 0;
    my_blinker[0] = 0;
    my_blinker[1] = 0;

    //my_idleFuelConsumption = 2500 * 3600; //Wh
    my_idleFuelConsumption = 0; //Wh
    my_fuelConsumptionAtVelocity = 0.2; //Wh/m
    my_totalFuelConsumption = 0;
    my_totalEmissions = 0;

    my_stoplineCenter = -1;

    my_currentPosition[x] = lane_->startingPosition()[x];
    my_currentPosition[y] = lane_->startingPosition()[y];

    my_currentVelocity[x] = my_maxSpeed * lane_->unitVector()[x];
    my_currentVelocity[y] = my_maxSpeed * lane_->unitVector()[y];

    my_unitVector[x] = lane_->unitVector()[x];
    my_unitVector[y] = lane_->unitVector()[y];

    my_currentAcceleration[x] = 0;
    my_currentAcceleration[y] = 0;

    my_path = path_;
    my_state = DRIVING;

    my_direction = lane_->laneDirection();
    my_laneNumber = lane_->number();

    my_runningLight = false;

    if(my_direction == NORTH || my_direction ==SOUTH)
    {
        my_stopline = lane_->endingPosition()[y];
        my_dot = y;
    }
    else
    {
        my_stopline = lane_->endingPosition()[x];
        my_dot = x;
    }

    correctLane(lane_, true);

    if(my_path != STRAIGHT)
    {
        switch(my_direction)
        {
            case(NORTH):
            {
                if(my_path == LEFT)
                {
                    my_modifier[x] = positive_sin;
                    my_modifier[y] = positive_cos;

                    my_turnRadius[x] = intersection_params.ns_left_turn_radii[x];
                    my_turnRadius[y] = intersection_params.ns_left_turn_radii[y];
                }
                else if (my_path == RIGHT)
                {
                    my_modifier[x] = negative_sin;
                    my_modifier[y] = positive_cos;

                    my_turnRadius[x] = intersection_params.ns_right_turn_radii[x];
                    my_turnRadius[y] = intersection_params.ns_right_turn_radii[y];
                }
                else
                {
                    SWERRINT(my_path);
                }
                
            }
                break;
            case(SOUTH):
            {
                if(my_path == LEFT)
                {
                    my_modifier[x] = negative_sin;
                    my_modifier[y] = negative_cos;

                    my_turnRadius[x] = intersection_params.ns_left_turn_radii[x];
                    my_turnRadius[y] = intersection_params.ns_left_turn_radii[y];
                }
                else if (my_path == RIGHT)
                {
                    my_modifier[x] = negative_sin;
                    my_modifier[y] = negative_cos;

                    my_turnRadius[x] = intersection_params.ns_right_turn_radii[x];
                    my_turnRadius[y] = intersection_params.ns_right_turn_radii[y];
                }
                else
                {
                    SWERRINT(my_path);
                }
            }
                break;
            case(EAST):
            {
                if(my_path == LEFT)
                {
                    my_modifier[x] = negative_cos;
                    my_modifier[y] = positive_sin;

                    my_turnRadius[x] = intersection_params.ew_left_turn_radii[x];
                    my_turnRadius[y] = intersection_params.ew_left_turn_radii[y];
                }
                else if (my_path == RIGHT)
                {
                    my_modifier[x] = negative_cos;
                    my_modifier[y] = positive_sin;

                    my_turnRadius[x] = intersection_params.ew_right_turn_radii[x];
                    my_turnRadius[y] = intersection_params.ew_right_turn_radii[y];
                }
                else
                {
                    SWERRINT(my_path);
                }
            }
                break;
            case(WEST):
            {
                if(my_path == LEFT)
                {
                    my_modifier[x] = positive_cos;
                    my_modifier[y] = negative_sin;

                    my_turnRadius[x] = intersection_params.ew_left_turn_radii[x];
                    my_turnRadius[y] = intersection_params.ew_left_turn_radii[y];
                }
                else if (my_path == RIGHT)
                {
                    my_modifier[x] = positive_cos;
                    my_modifier[y] = positive_sin;

                    my_turnRadius[x] = intersection_params.ew_right_turn_radii[x];
                    my_turnRadius[y] = intersection_params.ew_right_turn_radii[y];
                }
                else
                {
                    SWERRINT(my_path);
                }
            }
                break;
            default: SWERRINT(my_direction);
        }
    }
    draw(true);
    if (simulation_params.print_vehicle_info)
    {
        std::string filename = "./Output/VehicleOutput/Vehicle" + std::to_string((int)my_number) + ".txt";
        info.open(filename);
        printStartingInformation();
        printStep();
    }
}

SelfDrivingCar::~SelfDrivingCar()
{
    if (simulation_params.print_vehicle_info && info.is_open())
    {
        info.close();
    }
}

bool SelfDrivingCar::accelerate()
{
    bool acceleration_complete = false;
    if((my_state & ACCELERATING) && (my_state & DECELERATING))
    {   
        //hard swerr
        SWERRINT(my_state);
        return true;
    }

    bool dot = findComponent(my_exteriorPosition[FRONT_BUMPER], my_exteriorPosition[BACK_BUMPER]);
    bool not_dot = !dot;

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

    if(my_state & ACCELERATING)
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
        if(velocity_magnitude >= my_maxSpeed)
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
                acceleration_complete = true;
        }
    }
    else if(my_state & DECELERATING)
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
        if((velocity_magnitude - my_accelerationMagnitude * simulation_params.time_step) <= 0.0001)
            {
                my_currentVelocity[dot] = 0;
                my_currentVelocity[not_dot] = 0;
                my_accelerationMagnitude = 0;
                my_currentAcceleration[dot] = 0;
                my_currentAcceleration[not_dot] = 0;
                acceleration_complete = true;
            }
    }
    else
    {
        SWERRINT(my_state);
        return true;
    }
    if (acceleration_complete)
    {
        if(my_accelerationMagnitude != 0)
        {
            SWERRFLOAT(my_accelerationMagnitude);
        }
    }
    return acceleration_complete;
}

void SelfDrivingCar::accelerate(float acceleration_magnitude_)
{
    if(acceleration_magnitude_ < 0)
    {
        SWERRFLOAT(acceleration_magnitude_);
        acceleration_magnitude_ *= -1;
    }
    if(acceleration_magnitude_ == 0)
    {
        my_accelerationMagnitude = my_driver->comfortableAcceleration();
    }
    else
    {
        my_accelerationMagnitude = acceleration_magnitude_;
    }
}

bool SelfDrivingCar::sendCommand(command* vehicle_command_)
{
    if(vehicle_command_->vehicle_number != my_number)
    {
        SWERRINT(vehicle_command_->vehicle_number);
        return false;
    }
    my_nextCommand.command_type = vehicle_command_->command_type;
    my_nextCommand.value = vehicle_command_->value;
    my_nextCommand.vehicle_number = vehicle_command_->vehicle_number;
    if((my_nextCommand.command_type == vehicle_command_->command_type) &&
       (my_nextCommand.value == vehicle_command_->value) &&
       (my_nextCommand.vehicle_number == vehicle_command_->vehicle_number))
    {
        return true;
    }
    else
    {
        SWERRINT(my_nextCommand.command_type);
        SWERRFLOAT(my_nextCommand.value);
        SWERRINT(my_nextCommand.vehicle_number);
        return false;
    }
}

bool SelfDrivingCar::removeCommand()
{
    my_nextCommand.command_type = NULL_COMMAND;
    my_nextCommand.value = 0; //doesn't really matter
    my_nextCommand.vehicle_number = my_number; //doesn't really matter

    if(my_nextCommand.command_type != NULL_COMMAND)
    {
        SWERRINT(my_nextCommand.command_type);
        return false;
    }
    else
    {
        return true;
    }
}

bool SelfDrivingCar::verifyAcceleration(float acceleration_)
{
    return acceleration_ <= my_driver->comfortableAcceleration();
}

bool SelfDrivingCar::verifyDeceleration(float deceleration_)
{
    return deceleration_ <= my_driver->comfortableDeceleration();
}

bool SelfDrivingCar::forceRunLight()
{
    my_runningLight = true;
    return true;
}

bool SelfDrivingCar::ignore()
{
    return my_ignoreStatus;
}

void SelfDrivingCar::consumeFuel()
{
    float velocity_magnitude = MAGNITUDE(my_currentVelocity[x], my_currentVelocity[y]);
    if(velocity_magnitude == 0 && (my_currentVelocity[x] != 0 || my_currentVelocity[y] != 0))
    {
        SWERRFLOAT(my_currentVelocity[x] != 0 ? my_currentVelocity[x] : my_currentVelocity[y]);
    }
    //28% of Ontario's power grid is natural gas (fossil fuel based)
    //1Wh of electricity takes 92.67316535mL of natural gas (https://www.ocean.washington.edu/courses/envir215/energynumbers.pdf)
    float electricity_to_gas_modifier = 0.28 * 92.67317;
    float gas_to_kgCO2 = 1.94583997 / 1000000; //kg/ml
    float fuel_used_at_step = electricity_to_gas_modifier * (((my_fuelConsumptionAtVelocity * velocity_magnitude) + my_idleFuelConsumption) * simulation_params.time_step);
    my_totalFuelConsumption += fuel_used_at_step;
    my_totalEmissions += gas_to_kgCO2 * fuel_used_at_step;
}