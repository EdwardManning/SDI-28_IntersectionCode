#include "./Intersection.h"

//Function definitions for Intersection.h

/*
*   Name: Intersection
*
*   Description: Sets the intersection information and builds
*                and stores the roads in the my_roads array
*
*   Input: N/A
*
*   Output: N/A
*
*/
Intersection::Intersection()
{
    //The following four lines set the road in each direction
    //which is currently a pointer of a road into a pointer
    //"pointing at" a specific type of road.
    my_roads[NORTH] = new NorthRoad();
    my_roads[SOUTH] = new SouthRoad();
    my_roads[EAST] = new EastRoad();
    my_roads[WEST] = new WestRoad();
}

/*
*   Name: getRoad
*
*   Description: Returns a road based on the road number requested.
*
*   Input:  road_number_ -> the number of the road requested
*
*   Output: the road that is requested
*
*/
Road* Intersection::getRoad(uint8 road_number_)
{
    if (road_number_ < TOTAL_DIRECTIONS) //if the road is in the road list
    {
        return my_roads[road_number_]; //returns the requested road
    }
    else
    {
        SWERRINT(road_number_); //if the road does not exist
    }
    return my_roads[NORTH]; //only happens with swerr
}

/*
*   Name: getRoad
*
*   Description: Returns a road based on the road direction requested.
*
*   Input: road_direction_ -> The direction of the road requested
*
*   Output: the road that is requested
*
*/
Road* Intersection::getRoad(direction road_direction_)
{
    switch (road_direction_)
    {
        case(NORTH): return my_roads[NORTH];
            break;
        case(SOUTH): return my_roads[SOUTH];
            break;
        case(EAST): return my_roads[EAST];
            break;
        case(WEST): return my_roads[WEST];
            break;
        default: SWERRINT(road_direction_); //if the direction isn't north, south, east, or west (is not allowed)
    }
    return my_roads[NORTH]; //only happens with swerr
}

