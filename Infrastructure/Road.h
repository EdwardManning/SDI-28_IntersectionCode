#pragma once
#include "../Common/CommonDefs.h"
#include "./Lane.h"
#include "./EntryLane.h"
#include "./ExitLane.h"

/*
*   Name: Road
*
*   Description: Defines all of the shared functions and variables for the roads.
*
*   Type: Base Class
*
*   Inherited by: NorthRoad, SouthRoad, EastRoad, WestRoad
*/
class Road
{
public:
    Road();
    uint32 vehicleAtIndex(uint32 index_);
    uint32 indexOfVehicle(uint32 vehicle_number_);
    bool inRoad(uint32 vehicle_number_);
    void addToRoad(uint32 vehicle_number_);
    void addToRoad(uint32 vehicle_number_, uint32 index_);
    void removeFromRoad();
    bool removeFromRoad(uint32 vehicle_number_);
    uint32 numberOfVehicles();
    Lane* getLane(uint8 lane_number_);
    uint8 totalLanes();
    uint16 startingPosition();
    uint16 endingPosition();
    std::string name();
    direction roadDirection();
    uint8 speedLimit();
    virtual direction correspondingExit(path path_);
protected:
    Lane** my_laneList; //list of all the lanes in the road
    std::vector<uint32> my_vehicles;
    uint16 my_startingPosition; //where the road begins (which side of the frame)
    uint16 my_endingPosition; //where the road ends (which side of the intersection)
    direction my_direction; //the direction of the road
    std::string my_name; //the name of the road
    uint8 my_totalLanes; //the total number of lanes in the road
    uint8 my_speedLimit; //the speed limit of the road
private:
};