#include "./EntryLane.h"

//Function definitions for EntryLane.h

/*
*   Name: EntryLane
*
*   Description: Assigns values to protected variables.
*
*   Input: N/A
*
*   Output: N/A
*
*/
EntryLane::EntryLane(uint8 number_, direction direction_, path path_)
{
    //assign values to protected variables
    my_width = intersection_params.lane_width;
    my_length = intersection_params.lane_length;
    my_number = number_;
    my_laneDirection = direction_;
    my_lanePath = path_;
    my_name = DIRECTION_STR[my_laneDirection] + " " + PATH_STR[my_lanePath] + " " + LANE_STR;
    my_laneType = ENTRY_LANE;
    
    //certain variables are direction specific
    //the following switch statement is used to 
    //assign values to these variables
    switch(my_laneDirection)
    {
        case(NORTH):
        { 
            my_speedLimit = intersection_params.ns_speed_limit;

            //north centerline is measured left to right if when looking from the north
            //therefore right to left looking from normal (south) perspective
            //found by taking the frame width and subtracting the distance from the side
            // of the frame to the intersection (one lane length) and then subtracting
            //the width of all previous lanes, the corner of the intersection, the median
            //and half of the current lane.
            my_centerLine = intersection_params.frame_width - 
                            intersection_params.lane_length - 
                            ((my_number * intersection_params.lane_width) + 
                            intersection_params.ns_median_width + 
                            intersection_params.corner_width + 
                            (my_width / 2));
            
            my_startingPosition[x] = my_centerLine;
            my_endingPosition[x] = my_centerLine;

            my_startingPosition[y] = 0;
            my_endingPosition[y] = intersection_params.lane_length;
        }
            break;
        case(SOUTH):
        { 
            my_speedLimit = intersection_params.ns_speed_limit;

            //south centerline is measured left to right if when looking from the south
            //found by taking the width from the side to the intersection (one lane length)
            //and adding the width of all previous lanes, the corner of the intersection, 
            //the median and half the current lane.
            my_centerLine = intersection_params.lane_length + 
                            ((my_number * intersection_params.lane_width) + 
                            intersection_params.ns_median_width +
                            intersection_params.corner_width + 
                            (my_width / 2));

            my_startingPosition[x] = my_centerLine;
            my_endingPosition[x] = my_centerLine;

            my_startingPosition[y] = intersection_params.frame_length;
            my_endingPosition[y] = intersection_params.frame_length - intersection_params.lane_length;
        }
            break;
        case(EAST):
        { 
            my_speedLimit = intersection_params.ew_speed_limit;

            //east centerline is measured left to right if when looking from the east
            //therefore bottom to top looking from normal (south) perspective
            //found by taking the frame length and subtracting the distance from the side
            // of the frame to the intersection (one lane length) and then subtracting
            //the width of all previous lanes, the corner of the intersection, the median
            //and half of the current lane.
            my_centerLine = intersection_params.frame_length -
                            intersection_params.lane_length -
                            ((my_number * intersection_params.lane_width) + 
                            intersection_params.ew_median_width +
                            intersection_params.corner_width + 
                            (my_width / 2));
            
            my_startingPosition[y] = my_centerLine;
            my_endingPosition[y] = my_centerLine;

            my_startingPosition[x] = intersection_params.frame_width;
            my_endingPosition[x] = intersection_params.frame_width - intersection_params.lane_length;
        }
            break;
        case(WEST): 
        { 
            my_speedLimit = intersection_params.ew_speed_limit;

            //west centerline is measured left to right if when looking from the west
            //therefore top to bottom looking from normal (south) perspective
            //found by taking the length from the side to the intersection (one lane length)
            //and adding the width of all previous lanes, the corner of the intersection, 
            //the median and half the current lane.
            my_centerLine = intersection_params.lane_length +
                            ((my_number * intersection_params.lane_width) + 
                            intersection_params.ew_median_width +
                            intersection_params.corner_width + 
                            (my_width / 2));

            my_startingPosition[y] = my_centerLine;
            my_endingPosition[y] = my_centerLine;

            my_startingPosition[x] = 0;
            my_endingPosition[x] = intersection_params.lane_length;
        }
            break;
        default: SWERRINT(my_laneDirection); //this should never be hit
    }
}