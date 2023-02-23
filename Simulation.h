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
    
    bool light_change_occured;
    uint32 my_vehiclesMade;
    double elapsed_time; //Total time the simulation has been running
    Intersection my_intersection; //Holds the intersection information
    Vehicle* car; //Will be changed to a list later

    Vehicle** vehicle_list; //list of all vehicles (active and inactive)
    std::vector<Vehicle*> active_vehicles;

    std::ofstream events; //used for printing events
    std::ofstream results; //used for printing results at end of simulation
};