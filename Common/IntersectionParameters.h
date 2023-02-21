#pragma once
#include "./CommonTypes.h"
#include <cmath>


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
    uint8 ns_number_of_exits = 3;
    uint8 ns_median_width = 2; //in m (must be even number)
    //ew road params
    uint8 ew_speed_limit = 15; //in m/s
    uint8 ew_number_of_entries = 4;
    uint8 ew_number_of_exits = 3;
    uint8 ew_median_width = 2; //in m (must be even number)
    //ns light parameters
    uint8 ns_green_time = 15;
    uint8 ns_yellow_time = 3;
    uint8 ns_red_overlap_time = 2;
    bool n_advanced_green = true;
    bool s_advanced_green = true;
    uint8 ns_advanced_green_time = 6;
    uint8 ns_advanced_yellow_time = 3;
    uint8 ns_advanced_red_time = 1;
    //ew light parameters
    uint8 ew_green_time = 15;
    uint8 ew_yellow_time = 3;
    uint8 ew_red_overlap_time = 2;
    bool e_advanced_green = true;
    bool w_advanced_green = true;
    uint8 ew_advanced_green_time = 6;
    uint8 ew_advanced_yellow_time = 3;
    uint8 ew_advanced_red_time = 1;
    //intersection params (calculated not inputed)
    float ns_left_turn_radii[2];
    float ns_right_turn_radii[2];
    float ew_left_turn_radii[2];
    float ew_right_turn_radii[2];
    uint8 intersection_length;
    uint8 intersection_width;
    uint16 frame_length;
    uint16 frame_width;
    uint16 center_coordinates[2];

private:
    void calculateIntersectionInformation();
};