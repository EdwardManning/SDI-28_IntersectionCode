#include "./WestRoad.h"

//Function definitions for WestRoad.h

/*
*   Name: WestRoad
*
*   Description: Assigns values to variables and creates lanes.
*
*   Input: N/A
*
*   Output: N/A
*
*/
WestRoad::WestRoad()
{
    if (my_laneList == nullptr) //it should not be a nullptr (two dimensional)
    {
        SWERRINT(0);
    }

    //assign values to protected variables
    my_direction = WEST;
    my_totalLanes = intersection_params.ew_number_of_entries + intersection_params.ew_number_of_exits;
    my_laneList = new Lane*[my_totalLanes];
    my_name = DIRECTION_STR[WEST] + " " + ROAD_STR;
    my_speedLimit = intersection_params.ew_speed_limit;
    my_startingPosition = 0;
    my_endingPosition = intersection_params.lane_length;

    //populate laneList with lanes
    for (uint8 i = 0; i < my_totalLanes; i++)
    {
        if (i < intersection_params.ew_number_of_exits) //must be defining an exit lane
        {
            //all exit lanes are defined the same
            my_laneList[i] = new ExitLane(i, WEST, NULL_PATH);
        }
        else //we are now defining entry lanes
        {
            if(intersection_params.ew_number_of_entries == 1)
            {
                //if theres only one lane, you must be able to go any direction from that lane
                my_laneList[i] = new EntryLane(i, WEST, ALL_PATHS);
            }
            else if ( intersection_params.ew_number_of_entries == 2)
            {
                //if there are two lanes both must go straight but the left must also go left and the right must also go right
                my_laneList[i] = new EntryLane(i, WEST, (i - intersection_params.ew_number_of_exits) % 2 ? STRAIGHT_LEFT : STRAIGHT_RIGHT);
            }
            else 
            {
                //this means there are more than two lanes and we can specialize each lane
                uint8 current_entry = i - intersection_params.ew_number_of_exits;
                if (current_entry == 0)
                {
                    //the furthest left lane must go left
                    my_laneList[i] = new EntryLane(i, WEST, LEFT);
                }
                else if (i + 1 == my_totalLanes)
                {
                    //furthest right lane must go right
                    my_laneList[i] = new EntryLane(i, WEST, RIGHT);
                }
                else
                {
                    //all lanes that aren't furthest left or right must go straight
                    my_laneList[i] = new EntryLane(i, WEST, STRAIGHT);
                }
            }
        }
    }
}

/*
*   Name: correspondingExit
*
*   Description: Determines the exit direction given the path.
*
*   Input: path_ -> The path the vehicle is taking.
*
*   Output: The direction of the exit.
*
*/
direction WestRoad::correspondingExit(path path_)
{
    switch(path_)
    {
        case(LEFT): return NORTH;
            break;
        case(RIGHT): return SOUTH;
            break;
        case(STRAIGHT): return EAST;
            break;
        default: SWERRINT(path_);
    }
    return NORTH;
}