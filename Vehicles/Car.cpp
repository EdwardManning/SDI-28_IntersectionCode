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
    
    correctLane(lane_);
}