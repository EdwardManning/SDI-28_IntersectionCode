#pragma once
#include "../Common/CommonDefs.h"
#include "../Infrastructure/Lane.h"

/*
*   Name: Vehicle
*
*   Description: Defines all shared functions and variables for the vehicles.
*
*   Type: Base Class
*
*   Inherited by: Car
*
*/
class Vehicle
{
public:
    Vehicle();
    Vehicle(uint16 number_, path path_, Lane* lane_, DriverType driver_type_);

    //important functions
    void drive();
    void accelerate();
    void changeLane(path direction_);
    void stopLaneChange();
    bool correctLane(Lane* lane_);

    //less important functions
    void changeState(state state_, const bool adding_);
    void setLane(uint8 lane_number_);
    float* currentPosition();
    float* currentVelocity();
    float* currentAcceleration();
    uint16 number();
    std::string name();
    VehicleType vehicleType();
    path vehiclePath();
    uint8 currentState();
    DriverType driverType();
    direction vehicleDirection();
    uint8 laneNumber();
    uint8 maxSpeed();
    float timeInIntersection();
protected:
    float my_currentPosition[2]; //the current position of the vehicle
    float my_currentVelocity[2]; //the current velocity of the vehicle
    float my_currentAcceleration[2]; //the current acceleration of the vehicle
    uint16 my_number; //the vehicle number
    std::string my_name; //the name of the vehicle
    VehicleType my_vehicleType; //the type of vehicle
    path my_path; //the path the vehicle is taking
    uint8 my_state; //the current state of the vehicle
    DriverType my_driverType; //the driver type of the vehicle
    direction my_direction; //the direction the vehicle started from
    uint8 my_laneNumber; //the number of the lane the vehicle is currently in
    uint8 my_maxSpeed; //the maximum speed of the vehicle
    float my_timeInIntersection; //the amount of time spent in the intersection
private:
};