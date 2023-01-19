#include "./IntersectionParameters.h"

//Function definitions for IntersectionParameters.h

/*
*   Name: IntersectionParameters
*
*   Description: Calls the function that calculates the remaining parameters.
*
*   Input: N/A
*
*   Output: N/A
*
*/
IntersectionParameters::IntersectionParameters()
{
    calculateIntersectionInformation();
}

/*
*   Name: calculateIntersectionInformation
*
*   Description: Calculates the intersection information that is not given.
*                The parts it is calculating are denoted by the
*                Intersection Params (calculated not given) comment in the
*                IntersectionParameters class.
*
*   Input: N/A
*
*   Output: N/A
*
*/
void IntersectionParameters::calculateIntersectionInformation()
{
    //Length is verticle on your screen, width is horizontal

    //The length of the intersection is the width of the east/west lane (since the lane itself is horizontal, making its' width verticle)
    //the intersection length is then equal to the width of the median + the width of both corners + the width of all of the lanes
    intersection_length = ew_median_width +(2 * corner_width) +
                                      ( lane_width * (ew_number_of_entries + 
                                      ew_number_of_exits) );
    
    //The width of the intersection is the width of the north/south lane (since the lane itself is verticle, therefore the width is horizontal)
    //The intersection width is calculated using the same formula as the length above
    intersection_width = ns_median_width +(2 * corner_width) +
                                      ( lane_width * (ns_number_of_entries + 
                                      ns_number_of_exits) );

    //The frame length is the total length of the intersection system which is equal to the intersection length + the length of the north and south lanes
    frame_length = intersection_length + (2 * lane_length);

    //The frame width is the total width of the intersection system, calculated the same way as the length just using the east/west measurements
    frame_width = intersection_width + (2 * lane_length);

    //Since the intersection is not necessarily equal in length/width we need to calculate it's centerpoint
    if (intersection_width % 2 == 0)
    {
        //if the intersection's width is even then the centerpoint at x is simply half the frame width (the lanes are always equal in length)
        center_coordinates[0] = frame_width / 2; 
    }
    else
    {
        //if the width is not equal then it must be calculated by getting the adding one to half of the frame width -1
        //i.e. 5 / 2 = 2.5 (not allowed) therefore ((5 - 1) / 2) + 1 = 3 (which is proper)
        center_coordinates[0] = ( (frame_width - 1) / 2 ) + 1;
    }

    //the same logic is applied to the length as it was to the width
    if (intersection_length % 2 == 0)
    {
        center_coordinates[1] = frame_length / 2;
    }
    else
    {
        center_coordinates[1] = ( (frame_length - 1) / 2 ) + 1;
    }
}