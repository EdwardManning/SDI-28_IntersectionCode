#pragma once
#include "../Common/CommonDefs.h"
#include "./Road.h"
#include "./NorthRoad.h"
#include "./SouthRoad.h"
#include "./EastRoad.h"
#include "./WestRoad.h"

/*
*   Name: Intersection
*
*   Description: Builds and stores the intersection. Starts by
*                calculating the necessary intersection parameters
*                then builds and stores the four roads.
*
*   Type: Regular Class
*/

class Intersection
{
public:
    Intersection();
    Road* getRoad(uint8 road_number_);
    Road* getRoad(direction road_direction_);
private:
    Road* my_roads[TOTAL_DIRECTIONS]; //array of roads
    //add vehicles list and traffic light
};