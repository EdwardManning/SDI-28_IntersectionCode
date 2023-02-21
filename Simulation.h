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

    //printing functions (note that changeState() is a partial printing function but not listed here)
    void printResults();
    void printCompletion(Vehicle* vehicle_);
    void printLaneChange(Vehicle* vehicle_, uint8 new_lane_);
    void printTrafficLightStateChange(TrafficLight* traffic_light_);
    
    double elapsed_time; //Total time the simulation has been running
    Intersection my_intersection; //Holds the intersection information
    Vehicle* car; //Will be changed to a list later
    std::ofstream events; //used for printing events
    std::ofstream results; //used for printing results at end of simulation
};