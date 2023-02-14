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
Car::Car(uint16 number_, path path_, Lane* lane_, DriverType driver_type_)
{
    my_number = number_;
    my_vehicleType = CAR;
    my_driverType = driver_type_;
    my_name = std::to_string(my_number) + " " + VEHICLE_TYPE_STR[my_vehicleType] + " " + DRIVER_TYPE_STR[my_driverType];
    my_maxSpeed = lane_->speedLimit() * DRIVER_TYPE_MODIFIER[my_driverType];
    my_timeInIntersection = 0;
    my_totalTime = 0;

    my_currentPosition[x] = lane_->startingPosition()[x];
    my_currentPosition[y] = lane_->startingPosition()[y];

    my_currentVelocity[x] = my_maxSpeed * lane_->unitVector()[x];
    my_currentVelocity[y] = my_maxSpeed * lane_->unitVector()[y];

    my_currentAcceleration[x] = 0;
    my_currentAcceleration[y] = 0;

    my_path = path_;
    my_state = DRIVING;

    my_direction = lane_->laneDirection();
    my_laneNumber = lane_->number();

    if(my_direction == NORTH || my_direction ==SOUTH)
    {
        my_stopline = lane_->endingPosition()[y];
    }
    else
    {
        my_stopline = lane_->endingPosition()[x];
    }
    
    correctLane(lane_);

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
                    my_modifier[x] = positive_sin;
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
                    my_modifier[y] = negative_sin;

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
}