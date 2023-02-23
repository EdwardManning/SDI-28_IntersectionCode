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

    my_trafficLight = new TrafficLight(false);
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

uint32 Intersection::vehicleAtIndex(uint32 index_)
{
    try
    {
        if(index_ < my_vehicles.size() && index_ >= 0)
        {
            return my_vehicles[index_];
        }
        else
        {
            throw(std::out_of_range("Out of Vehicle List Bounds"));
        }
    }
    catch (const std::out_of_range &Out_of_Range)
    {
        //hard SWERR
        SWERRINT(index_);
        throw;
    }
}

uint32 Intersection::indexOfVehicle(uint32 vehicle_number_)
{
    if (my_vehicles.size() > 0)
    {
        for(uint32 i = 0; i < my_vehicles.size(); i++)
        {
            if (vehicle_number_ == my_vehicles[i])
            {
                return i;
            }
        }
    }
    SWERRINT(-1);
    return 0;
}

bool Intersection::inIntersection(uint32 vehicle_number_)
{
    if (my_vehicles.size() > 0)
    {
        for(uint32 i = 0; i < my_vehicles.size(); i++)
        {
            if (vehicle_number_ == my_vehicles[i])
            {
                return true;
            }
        }
    }
    return false;
}

void Intersection::addToIntersection(uint32 vehicle_number_)
{
    if(!inIntersection(vehicle_number_))
    {
        my_vehicles.insert(my_vehicles.begin(), vehicle_number_);
    }
    else
    {
        SWERRINT(vehicle_number_);
    }
}

void Intersection::addToIntersection(uint32 vehicle_number_, uint32 index_)
{
    if(!inIntersection(vehicle_number_))
    {
        my_vehicles.insert(my_vehicles.begin() + index_, vehicle_number_);
    }
    else
    {
        SWERRINT(vehicle_number_);
    }
}

void Intersection::removeFromIntersection()
{
    my_vehicles.pop_back();
}

bool Intersection::removeFromIntersection(uint32 vehicle_number_)
{
    if (inIntersection(vehicle_number_))
    {
        my_vehicles.erase(my_vehicles.begin() + indexOfVehicle(vehicle_number_));
        return true;
    }
    return false;
}

TrafficLight* Intersection::trafficLight()
{
    return my_trafficLight;
}

uint32 Intersection::numberOfVehicles()
{
    return my_vehicles.size();
}

