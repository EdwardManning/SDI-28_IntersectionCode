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
    uint16 my_startingPosition; //where the road begins (which side of the frame)
    uint16 my_endingPosition; //where the road ends (which side of the intersection)
    direction my_direction; //the direction of the road
    std::string my_name; //the name of the road
    uint8 my_totalLanes; //the total number of lanes in the road
    uint8 my_speedLimit; //the speed limit of the road
private:
};