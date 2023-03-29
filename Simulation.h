#pragma once

#include <iostream>
#include <fstream>

#include "Common/CommonDefs.h"
#include "Infrastructure/Intersection.h"
#include "Vehicles/Car.h"
#include "Vehicles/SelfDrivingCar.h"

/*
*   Name: Simulation
*
*   Description: Controls and runs the simulation.
*
*   Type: Regular Class
*/
class Simulation
{
public:
    Simulation();
    ~Simulation();
private:
    void run(); 
    bool completionCheck(); 
    void driverPerformActions(Vehicle* vehicle_);
    void vehiclePerformActions(Vehicle* autonomous_vehicle_);
    bool passedStopLine(Vehicle* vehicle_);
    void changeState(Vehicle* vehicle_, state state_, const bool adding_);
    bool passedExitStartLine(Vehicle* vehicle_);
    void setLane(Vehicle* vehicle_);
    void debugIntersection();
    uint8 closestLane(float checked_position_, Road* road_);
    path changeLaneDirection(Vehicle* vehicle_);
    bool overCenterLine(Vehicle* vehicle_, path lane_change_direction_);
    bool vehicleCompleted(Vehicle* vehicle_);
    void generateVehicle(uint32 number_);
    bool spawnVehicle();
    bool shoulderCheck(Vehicle* vehicle_, path lane_change_direction_);
    bool collisionAnalysis();
    void calculateAverages();
    void calculateTotalAverages(Vehicle* vehicle_);
    void calculateLeftAverages(Vehicle* vehicle_);
    void calculateStraightAverages(Vehicle* vehicle_);
    void calculateRightAverages(Vehicle* vehicle_);
    severity calculateCollisionSeverity(Vehicle* first_vehicle_, Vehicle* second_vehicle_);
    direction opposingDirection(direction direction_);

    //acceleration code
    //{
    void accelerate(Vehicle* vehicle_);
    //{
    bool checkTurnClear(Vehicle* vehicle_);
    float determineCloseProximityDecelerationDistance(Vehicle* vehicle_);
        //{
    float determineCloseVehicleDecelerationDistance(Vehicle* vehicle_);
            //{
    float distanceAhead(Vehicle* current_vehicle_, Vehicle* test_vehicle_);
            //}
    float determineLaneChangeDecelerationDistance(Vehicle* vehicle_);
            //{
    float determineChangingLaneDeceleration(Vehicle* vehicle_);
                //{
    bool laneChangeDecelerationRequired(Vehicle* vehicle_);
                //}
    float determineCloseLaneChangeDeceleration(Vehicle* vehicle_);
                //{
    float checkLaneBlinkerDistance(Vehicle* vehicle_, Lane* lane_, int8 direction_);
                //}
            //}
    float determineBrakeLightDecelerationDistance(Vehicle* vehicle_);
        //}
    float determineLightBasedDecelerationDistance(Vehicle* vehicle_);
        //{ 
    float lightChangeDecelerationDistance(Vehicle* vehicle_);
    float lightColourDecelerationDistance(Vehicle* vehicle_);
        //}
    //}
    void startAcceleration(Vehicle* vehicle_, float target_speed_);
    void startDeceleration(Vehicle* vehicle_, float target_speed_);
    bool startDeceleration(Vehicle* vehicle_, float target_speed_, float distance_remaining_);
    void changeDeceleration(Vehicle* vehicle_, float target_speed_);
    void changeDeceleration(Vehicle* vehicle_, float target_speed_, float distance_remaining_);
    //}

    //self driving acceleration code
    void sdv_accelerate(Vehicle* vehicle_);
    //{
    float sdv_closeProximityDeceleration(Vehicle* vehicle_);
        //{
    float sdv_determineCloseVehicleDeceleration(Vehicle* vehicle_);
    float sdv_determineLaneChangeDeceleration(Vehicle* vehicle_);
            //{
    float sdv_determineChangingLaneDecelereation(Vehicle* vehicle_);
    float sdv_determineCloseLaneChangeDeceleration(Vehicle* vehicle_);
                //{
    float sdv_determineLaneBlinkerRequiredAcceleration(Vehicle* vehicle_, Lane* lane_, int8 direction_);
                //}
            //}
    float sdv_determineBrakeLightDeceleration(Vehicle* vehicle_);
        //}
    float sdv_lightBasedDeceleration(Vehicle* vehicle_);
        //{
    float sdv_lightChangeDeceleration(Vehicle* vehicle_);
    float sdv_lightColourDeceleration(Vehicle* vehicle_);
        //}
    float sdv_controlSystemAcceleration(Vehicle* vehicle_);
    //}
    void sdv_startAcceleration(Vehicle* vehicle_);
    void sdv_startAcceleration(Vehicle* vehicle_, float acceleration_magnitude_);
    void sdv_startDeceleration(Vehicle* vehicle_, float deceleration_magnitude_);
    

    //active vehicles functions
    Vehicle* vehicleAtIndex(uint32 index_);
    uint32 indexOfVehicle(Vehicle* vehicle_);
    bool isActive(Vehicle* vehicle_);
    void addToActiveVehicles(Vehicle* vehicle_);
    bool removeFromActiveVehicles(Vehicle* vehicle_);

    //printing functions (note that changeState() is a partial printing function but not listed here)
    void printResults();
    void printResultsForPython();
    void printCompletion(Vehicle* vehicle_);
    void printLaneChange(Vehicle* vehicle_, uint8 new_lane_);
    void printTrafficLightStateChange(TrafficLight* traffic_light_);
    void printVehicleArrival(Vehicle* vehicle_);
    void printCollisionInformation(Vehicle* first_vehicle_, Vehicle* second_vehicle_);
    void printVehicleFailInformation(Vehicle* vehicle_);
    
    bool light_change_occured;
    uint32 my_vehiclesMade;
    uint32 my_selfDrivingVehiclesMade;
    uint32 my_leftVehiclesMade;
    uint32 my_sdvLeftVehiclesMade;
    uint32 my_straightVehiclesMade;
    uint32 my_sdvStraightVehiclesMade;
    uint32 my_rightVehiclesMade;
    uint32 my_sdvRightVehiclesMade;
    long double elapsed_time; //Total time the simulation has been running
    Intersection my_intersection; //Holds the intersection information
    Vehicle* car; //Will be changed to a list later
    float my_spawnTimer;

    //averages
    float average_time_between_spawn;
    float averages[TOTAL_AVERAGES];
    float left_averages[TOTAL_AVERAGES];
    float straight_averages[TOTAL_AVERAGES];
    float right_averages[TOTAL_AVERAGES];
    float self_driving_averages[TOTAL_AVERAGES];
    float sdv_left_averages[TOTAL_AVERAGES];
    float sdv_straight_averages[TOTAL_AVERAGES];
    float sdv_right_averages[TOTAL_AVERAGES];
    float human_driving_averages[TOTAL_AVERAGES];
    float hd_left_averages[TOTAL_AVERAGES];
    float hd_straight_averages[TOTAL_AVERAGES];
    float hd_right_averages[TOTAL_AVERAGES];

    Vehicle** vehicle_list; //list of all vehicles (active and inactive)
    std::vector<Vehicle*> active_vehicles;

    std::ofstream events; //used for printing events
    std::ofstream results; //used for printing results at end of simulation
    std::ofstream python_results;
    std::ofstream debug_log; //used for printing non-swerr "unusual" behaviour
};