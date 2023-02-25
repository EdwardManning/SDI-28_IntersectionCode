#pragma once

#include <fstream>

#include "../Common/CommonDefs.h"
#include "../Infrastructure/Lane.h"
#include "./Driver.h"
#include "./HumanDriver.h"

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
    Vehicle(uint32 number_, path path_, Lane* lane_, DriverType driver_type_);

    //important functions
    void drive();
    bool accelerate();
    void accelerate(float target_speed_);
    void changeLane(path direction_);
    void stopLaneChange();
    bool correctLane(Lane* lane_, bool initialization_ = false);
    void turn();
    void stopTurn(Lane* lane_);
    bool collisionCheck(Vehicle* vehicle_);
    bool lightChange(lightColour colour_);
    bool checkImportantPosition(Vehicle* vehicle_);
    void setMaxSpeed(float new_max_speed_);
    void setCurrentSeparation(float separation_);

    //less important functions
    void changeState(state state_, const bool adding_);
    void setLane(uint8 lane_number_);
    void completed();
    float* currentPosition();
    float* currentVelocity();
    float* currentAcceleration();
    uint32 number();
    std::string name();
    VehicleType vehicleType();
    path vehiclePath();
    uint8 currentState();
    DriverType driverType();
    direction vehicleDirection();
    uint8 laneNumber();
    uint8 maxSpeed();
    float timeInIntersection();
    float totalTime();
    float timeStopped();
    float* exteriorPosition(vehiclePoints vehicle_point_);
    float* exteriorPosition(uint8 vehicle_point_);
    bool isCompleted();
    int8* unitVector();
    float currentAccelerationMagnitude();
    bool blinker(path direction_);
    bool blinker(bool direction_);
    bool brakeLights();
    float mimumumStoppingDistance();
    float minimumFollowingDistance();
    float currentSeparation();

    void toggleBlinker(path direction_, bool on_);
    void toggleBrakeLights(bool on_);
    bool yellowLightAnalysis();
    bool yellowLightAnalysis(float current_acceleration_);
    bool redLightAnalysis();
    bool redLightAnalysis(float current_acceleration);
    void requestAccelerationAdjustment(float adjustment_, bool is_positive_ = true);
protected:
    void draw(bool initialization_ = false);
    void updateUnitVector();
    void adjustAccelerationMagnitude(float adjustment_, bool is_positive_);
    void adjustAccelerationMagnitude(float acceleration_magnitude_);

    //printing functions
    void printStartingInformation();
    void printStep();
    void printFinalInformation();
    
    float my_currentPosition[2]; //the current position of the vehicle
    float my_currentVelocity[2]; //the current velocity of the vehicle
    float my_currentAcceleration[2]; //the current acceleration of the vehicle
    uint32 my_number; //the vehicle number
    std::string my_name; //the name of the vehicle
    VehicleType my_vehicleType; //the type of vehicle
    path my_path; //the path the vehicle is taking
    uint8 my_state; //the current state of the vehicle
    direction my_direction; //the direction the vehicle started from
    uint8 my_laneNumber; //the number of the lane the vehicle is currently in
    uint8 my_maxSpeed; //the maximum speed of the vehicle
    float my_timeInIntersection; //the amount of time spent in the intersection
    float my_totalTime; //the amount of time it took to fully clear the intersection
    float my_stopTime; //the amount of time spent stopped
    modifier my_modifier[2]; //the modifier used to turn if necessary
    float my_turnRadius[2]; //the radius of the turn if necessary
    float my_stopline; //the end of the starting road
    float my_stoplineCenter; //the center of the lane at the stop point
    bool my_completionStatus; //true if completed intersection, false otherwise
    Driver* my_driver; //the driver of the vehicle
    float my_exteriorPosition[TOTAL_POINTS][TOTAL_DIMENSIONS]; //holds the current position of the exterior of the vehicle
    float my_maxDeceleration;
    float my_accelerationMagnitude;
    bool my_dot;
    int8 my_unitVector[2];
    float my_targetSpeed;
    bool my_blinker[2]; //left is 0, right is 1
    bool my_brakeLights;
    float my_currentSeparation;
    std::ofstream info;
private:
};