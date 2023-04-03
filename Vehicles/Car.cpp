#include "Car.h"

//Function definitions for Car.h 

/*
*   Name: Car
*
*   Description: Assigns values to protected variables.
*
*   Input: number_ -> The vehicle number.
*          path_ -> The path the vehicle will take.
*          lane_ -> Pointer to the lane the vehicle starts in.
*          driver_type_ -> The type of the driver.
*
*   Output: N/A
*
*/
Car::Car(uint32 number_, path path_, Lane* lane_, DriverType driver_type_)
{
    my_number = number_;
    my_vehicleType = CAR;
    my_driver = new HumanDriver(driver_type_);
    my_name = VEHICLE_TYPE_STR[my_vehicleType] + " " + std::to_string(my_number);
    my_maxSpeed = lane_->speedLimit() * my_driver->modifier();
    my_timeInIntersection = 0;
    my_totalTime = 0;
    my_stopTime = 0;
    my_timeAtMaxSpeed = 0;
    my_completionStatus = false;
    my_maxDeceleration = 7; //m/s (I looked it up and apparenly cars can usually decelerate this fast)
    my_accelerationMagnitude = 0;
    my_currentSeparation = -1;

    my_nextCommand.command_type = NULL_COMMAND;
    my_nextCommand.value = 0;
    my_nextCommand.vehicle_number = my_number;

    my_brakeLights = 0;
    my_blinker[0] = 0;
    my_blinker[1] = 0;

    switch(my_driver->driverType())
    {
        case(CALM):
        {
            my_idleFuelConsumption = 0.1389;
            my_fuelConsumptionAtVelocity = 0.05;
        }
            break;
        case(NORMAL):
        {
            my_idleFuelConsumption = 0.2778;
            my_fuelConsumptionAtVelocity = 0.065;
        }
            break;
        case(AGGRESSIVE):
        {
            my_idleFuelConsumption = 0.4167;
            my_fuelConsumptionAtVelocity = 0.08;
        }
            break;
        default: SWERRINT(my_driver->driverType());
    }
    my_totalFuelConsumption = 0;
    my_totalEmissions = 0;

    my_stoplineCenter = -1; //for swerr purposes

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

/*
*   Name: ~Car
*
*   Description: Performs necessary cleanup.
*
*   Input: N/A
*
*   Output: N/A
*
*/
Car::~Car()
{
    if (simulation_params.print_vehicle_info && info.is_open())
    {
        info.close();
    }
}