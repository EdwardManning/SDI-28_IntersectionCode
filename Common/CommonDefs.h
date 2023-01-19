#pragma once
#ifndef COMMON_DEFS
#define COMMON_DEFS
#include <string>
#include "./CommonTypes.h"
#include "./IntersectionParameters.h"

//for software error (SWERR) reporting purposes
#define SWERRINT(x) fprintf(stderr, "SWERR at %s:%d - (%d)\n", __FILE__, __LINE__, x)
#define SWERRSTR(x) fprintf(stderr, "SWERR at %s:%d - (%s)\n", __FILE__, __LINE__, x)


//for coordinates
const bool x = 0;
const bool y = 1;

//lane types
const bool ENTRY_LANE = 0;
const bool EXIT_LANE = 1;

//constant output strings
const std::string LANE_STR = "lane";
const std::string EXIT_STR = "exit";
const std::string ROAD_STR = "road";

//all directions (mostly for roads)
enum direction
{
    NORTH,            //0
    SOUTH,            //1
    EAST,             //2
    WEST,             //3
    TOTAL_DIRECTIONS, //4
};

//strings corresponding to directions
const std::string DIRECTION_STR[]
{
    "north",
    "south",
    "east",
    "west",
    "total_directions",
};

//all paths (mostly for lanes)
enum path
{
    NULL_PATH,        //0 used for exit lanes
    LEFT,             //1
    STRAIGHT,         //2
    RIGHT,            //3
    STRAIGHT_LEFT,    //4
    STRAIGHT_RIGHT,   //5
    ALL_PATHS,        //6
};

//strings corresponding to paths
const std::string PATH_STR[]
{
    "null",
    "left", 
    "straight",
    "right",
    "straight left",
    "straight right",
    "all path",
};

//constant variable holding all of the necessary intersection parameters
const IntersectionParameters intersection_params;

#endif