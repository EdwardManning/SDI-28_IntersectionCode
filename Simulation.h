#pragma once

#include <iostream>
#include <fstream>

#include "Common/CommonDefs.h"
#include "Infrastructure/Intersection.h"
#include "Vehicles/Car.h"

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
    severity calculateCollisionSeverity(Vehicle* first_vehicle_, Vehicle* second_vehicle_);
    direction opposingDirection(direction direction_);

    //acceleration code
    void determineAcceleration(Vehicle* vehicle_);
    bool closeVehicleDecelerationRequired(Vehicle* vehicle_);
    bool preIntersectionCloseDecelerationRequired(Vehicle* vehicle_); //part of close vehicle acceleration
    bool closeLaneChangeDecelerationRequired(Vehicle* vehicle_); //part of pre intersection close acceleration
    bool inIntersectionCloseDecelerationRequired(Vehicle* vehicle_); //part of close vehicle acceleration
    bool postIntersectionCloseDecelerationRequired(Vehicle* vehicle_); //part of close vehicle acceleration
    bool laneChangeDecelerationRequired(Vehicle* vehicle_);
    bool lightColourDecelerationRequired(Vehicle* vehicle_);
    bool lightChangeDecelerationRequired(Vehicle* vehicle_);
    bool maxSpeedDecelerationRequired(Vehicle* vehicle_);
    void startAcceleration(Vehicle* vehicle_, float target_speed_);
    void startDeceleration(Vehicle* vehicle_, float target_speed_);
    void startDeceleration(Vehicle* vehicle_, float target_speed_, float distance_remaining_);
    void changeDeceleration(Vehicle* vehicle_, float target_speed_);
    void changeDeceleration(Vehicle* vehicle_, float target_speed_, float distance_remaining_);
    bool checkLaneBlinkers(Vehicle* vehicle_, Lane* lane_, int8 direction_);
    bool scanAhead(Vehicle* vehicle_);
    bool preIntersectionScanAhead(Vehicle* vehicle_);
    bool checkIntersectionClear(Vehicle* vehicle_);
    bool conflictingPaths(Vehicle* current_vehicle_, Vehicle* test_vehicle_);
    bool vehicleAhead(Vehicle* vehicle_, bool through_intersection_);
    float vehicleSeparation(Vehicle* current_vehicle_, Vehicle* ahead_vehicle_);
    Vehicle* whichVehicleAhead(Vehicle* vehicle_, bool through_intersection_);

    //active vehicles functions
    Vehicle* vehicleAtIndex(uint32 index_);
    uint32 indexOfVehicle(Vehicle* vehicle_);
    bool isActive(Vehicle* vehicle_);
    void addToActiveVehicles(Vehicle* vehicle_);
    bool removeFromActiveVehicles(Vehicle* vehicle_);

    //printing functions (note that changeState() is a partial printing function but not listed here)
    void printResults();
    void printCompletion(Vehicle* vehicle_);
    void printLaneChange(Vehicle* vehicle_, uint8 new_lane_);
    void printTrafficLightStateChange(TrafficLight* traffic_light_);
    void printVehicleArrival(Vehicle* vehicle_);
    void printCollisionInformation(Vehicle* first_vehicle_, Vehicle* second_vehicle_);
    
    bool light_change_occured;
    uint32 my_vehiclesMade;
    long double elapsed_time; //Total time the simulation has been running
    Intersection my_intersection; //Holds the intersection information
    Vehicle* car; //Will be changed to a list later
    float my_spawnTimer;

    //averages
    float average_time_between_spawn;
    float averages[TOTAL_AVERAGES];

    Vehicle** vehicle_list; //list of all vehicles (active and inactive)
    std::vector<Vehicle*> active_vehicles;

    std::ofstream events; //used for printing events
    std::ofstream results; //used for printing results at end of simulation
};