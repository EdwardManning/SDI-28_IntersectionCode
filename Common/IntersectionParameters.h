#pragma once
#include "./CommonTypes.h"


/*
*   Name: IntersectionParameters
*
*   Description: Stores and defines the parameters of the intersection.
*
*   Type: Regular Class
*/
class IntersectionParameters
{
public:
    IntersectionParameters();
    //common road params
    uint8 lane_length = 100; //in m
    uint8 lane_width = 5; //in m
    uint8 corner_width = 1; //in m
    //ns road params
    uint8 ns_speed_limit = 15; //in m/s
    uint8 ns_number_of_entries = 4;
    uint8 ns_number_of_exits = 4;
    uint8 ns_median_width = 2; //in m (must be even number)
    //ew road params
    uint8 ew_speed_limit = 15; //in m/s
    uint8 ew_number_of_entries = 4;
    uint8 ew_number_of_exits = 4;
    uint8 ew_median_width = 2; //in m (must be even number)
    //intersection params (calculated not inputed)
    uint8 intersection_length;
    uint8 intersection_width;
    uint16 frame_length;
    uint16 frame_width;
    uint16 center_coordinates[2];
private:
    void calculateIntersectionInformation();
};