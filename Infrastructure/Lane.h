#pragma once
#include "../Common/CommonDefs.h"
#include <cstdlib>

/*
*   Name: Road
*
*   Description: Defines all of the shared functions and variables for the lanes.
*
*   Type: Base Class
*
*   Inherited by: EntryLane, ExitLane
*/
class Lane
{
public:
    Lane();
    Lane(uint8 number_, direction direction_, path path_);
    float* startingPosition();
    float* endingPosition();
    float centerLine();
    uint8 width();
    uint8 length();
    std::string name();
    uint8 number();
    direction laneDirection();
    path lanePath();
    bool laneType();
    uint8 speedLimit();
protected:
    //it is worth noting that entry and exit lanes begin and end at opposite
    //ends of the road
    //entry lanes start at the edge of the frame and end at the intersection
    //exit lanes start at the intersection and end at the edge of the frame
    float my_startingPosition[2]; //the coordinates where the lane begins
    float my_endingPosition[2]; //the coordinates where the lane ends
    
    float my_centerLine; //the center coordinate of the lane 
    uint8 my_width; //the width of the lane
    uint8 my_length; //the length of the lane
    std::string my_name; //the name of the lane
    uint8 my_number; //the lane number
    direction my_laneDirection; //the direction of the lane
    path my_lanePath; //the path of the lane
    bool my_laneType; //the type of the lane
    uint8 my_speedLimit; //the speed limit of the lane
private:
};