#pragma once
#ifndef COMMON_DEFS
#define COMMON_DEFS
#include <string>
#include <cmath>
#include "./CommonTypes.h"
#include "./IntersectionParameters.h"
#include "./SimulationParameters.h"

//for software error (SWERR) reporting purposes
#define SWERRINT(x) fprintf(stderr, "SWERR at %s:%d - (%d)\n", __FILE__, __LINE__, x)
#define SWERRFLOAT(x) fprintf(stderr, "SWERR at %s:%d - (%f)\n", __FILE__, __LINE__, x)
#define SWERRSTR(x) fprintf(stderr, "SWERR at %s:%d - (%s)\n", __FILE__, __LINE__, x)

#define PI  3.1415926535

//for vehicle states
//each state pertains to one bit in a byte
enum state
{
    DRIVING = 1,
    ACCELERATING = 2,
    DECELERATING = 4,
    CHANGING_LANES = 8,
    TURNING = 16,
    CORRECT_LANE = 32,
    IN_INTERSECTION = 64,
    THROUGH_INTERSECTION = 128,
};

//for state changes
const bool REMOVE = 0;
const bool ADD = 1;

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

//types of vehicles
enum VehicleType
{
    CAR,              //0
    SELF_DRIVING_CAR, //1
};

const std::string VEHICLE_TYPE_STR[]
{
    "car",
    "self-driving car",
};

//types of drivers 
enum DriverType
{
    NULL_DRIVER,   //0
    SLOW,          //1
    NORMAL,        //2
    AGGRESSIVE,    //3
};

const std::string DRIVER_TYPE_STR[]
{
    "no driver",
    "slow",
    "normal",
    "aggressive",
};

//normal and null_drivers have no modifier
//aggressive drivers drive faster and slow drivers slower
const float DRIVER_TYPE_MODIFIER[]
{
    1,
    0.95,
    1,
    1.05,
};

//direction vectors to set lane velocity modifiers
//will be used by vehicles to initialize velocity
//Note: direction in title relates to direction travelling not direction coming from
//for instance NORTH_VECTOR will be used to by south entry lanes as its' vector
const struct directionVectors
{
    const int8 NORTH_VECTOR[2] = { 0, -1 }; //travelling north
    const int8 SOUTH_VECTOR[2] = { 0, 1 }; //travelling south
    const int8 EAST_VECTOR[2] = { 1, 0 }; //travelling east
    const int8 WEST_VECTOR[2] = { -1, 0 }; //travelling west
}DIRECTION_VECTOR;

//constant variable holding all of the necessary parameters
const IntersectionParameters intersection_params;
const SimulationParameters simulation_params;

//functions used for turn modifiers
//used with the modifier type
static float positive_cos(float period)
{
    return cos(period);
}

static float positive_sin(float period)
{
    return sin(period);
}

static float negative_cos(float period)
{
    return -1 * cos(period);
}

static float negative_sin(float period)
{
    return -1 * sin(period);
}

//magnitude of a vector
constexpr float MAGNITUDE(float x_value, float y_value)
{
    return sqrt( ( pow(x_value, 2) ) + ( pow(y_value, 2) ) );
}

template <typename type>
type max(type a, type b)
{
    return (abs(a) > abs(b)) ? a : b;
}

template <typename type>
bool maxComponent(type x_component, type y_component)
{
    return (abs(x_component) > abs(y_component)) ? x : y;
}
#endif