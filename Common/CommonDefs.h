#pragma once
#ifndef COMMON_DEFS
#define COMMON_DEFS
#include <string>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <exception>

#include "./CommonTypes.h"
#include "./IntersectionParameters.h"
#include "./SimulationParameters.h"
#include "./VehicleParameters.h"
#include "./Exceptions.h"

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
const bool STOP = 0;

//for state changes
const bool REMOVE = 0;
const bool ADD = 1;

//for vehicle lights
const bool OFF = 0;
const bool ON = 1;

//for coordinates
const bool x = 0;
const bool y = 1;
const uint8 TOTAL_DIMENSIONS = 2; //there are only 2 dimensions, x and y

const bool NS = 0;
const bool EW = 1;

//lane types
const bool ENTRY_LANE = 0;
const bool EXIT_LANE = 1;

//constant output strings
const std::string LANE_STR = "lane";
const std::string EXIT_STR = "exit";
const std::string ROAD_STR = "road";
const std::string DRIVER_STR = "driver";
const std::string NOT_APPLICABLE = "N/A";

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
    CALM,          //1
    NORMAL,        //2
    AGGRESSIVE,    //3
};

const std::string DRIVER_TYPE_STR[]
{
    "no driver",
    "calm",
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

enum lightState
{
    NORTH_GREEN_LIGHT = 1,
    NORTH_YELLOW_LIGHT = 2,
    NORTH_ADVANCED_GREEN_LIGHT = 4,
    NORTH_ADVANCED_YELLOW_LIGHT = 8,
    SOUTH_GREEN_LIGHT = 16,
    SOUTH_YELLOW_LIGHT = 32,
    SOUTH_ADVANCED_GREEN_LIGHT = 64,
    SOUTH_ADVANCED_YELLOW_LIGHT = 128,
    EAST_GREEN_LIGHT = 256,
    EAST_YELLOW_LIGHT = 512,
    EAST_ADVANCED_GREEN_LIGHT = 1024,
    EAST_ADVANCED_YELLOW_LIGHT = 2048,
    WEST_GREEN_LIGHT = 4096,
    WEST_YELLOW_LIGHT = 8192,
    WEST_ADVANCED_GREEN_LIGHT = 16384,
    WEST_ADVANCED_YELLOW_LIGHT = 32768,
};

enum lightEventState
{
    NORTH_ADVANCED_GREEN,      //0
    SOUTH_ADVANCED_GREEN,      //1
    EAST_ADVANCED_GREEN,       //2
    WEST_ADVANCED_GREEN,       //3
    NORTH_ADVANCED_YELLOW,     //4
    SOUTH_ADVANCED_YELLOW,     //5
    EAST_ADVANCED_YELLOW,      //6
    WEST_ADVANCED_YELLOW,      //7
    NS_ADVANCED_GREEN,         //8
    EW_ADVANCED_GREEN,         //9
    NS_ADVANCED_YELLOW,        //10
    EW_ADVANCED_YELLOW,        //11
    NS_ADVANCED_RED,           //12
    EW_ADVANCED_RED,           //13
    NS_GREEN,                  //14
    EW_GREEN,                  //15
    NS_YELLOW,                 //16
    EW_YELLOW,                 //17
    NS_MUTUAL_RED,             //18 preceeds ns green or ns advanced greens
    EW_MUTUAL_RED,             //19 preceeds ns green or ns advanced greens
    //dedicated greens are used after solo advanced greens to prevent collisions
    NORTH_GREEN,               //20 
    SOUTH_GREEN,               //21
    EAST_GREEN,                //22
    WEST_GREEN,                //23
    TOTAL_LIGHT_EVENTS,        //24
};

const std::string LIGHT_EVENT_STATE_STR[]
{
    "north advanced green",
    "south advanced green",
    "east advanced green",
    "west advanced green",
    "north advanced yellow",
    "south advanced yellow",
    "east advanced yellow",
    "west advanced yellow",
    "north/south advanced green",
    "east/west advanced green",
    "north/south advanced yellow",
    "east/west advanced yellow",
    "north/south advanced red",
    "east/west advanced red",
    "north/south green",
    "east/west green",
    "north/south yellow",
    "east/west yellow",
    "north/south based mutual red",
    "east/west based mutual red",
    "north green",
    "south green",
    "east green",
    "west green",
    "total light events",
};

enum lightColour
{
    GREEN,   //0
    YELLOW,  //1
    RED,     //2
};

const std::string LIGHT_COLOUR_STR[]
{
    "green",
    "yellow",
    "red",
};

//vehicle coordinate points correspond to
//specific parts of the vehicle
//used for defining vehicles as 2D instead of as points
enum vehiclePoints
{
    FRONT_LEFT,     //0
    FRONT_BUMPER,   //1
    FRONT_RIGHT,    //2
    BACK_LEFT,      //3
    BACK_BUMPER,    //4
    BACK_RIGHT,     //5
    TOTAL_POINTS,   //6
};

static vehiclePoints getVehiclePoint(uint8 value)
{
    switch(value)
    {
        case(0): return FRONT_LEFT;
            break;
        case(1): return FRONT_BUMPER;
            break;
        case(2): return FRONT_RIGHT;
            break;
        case(3): return BACK_LEFT;
            break;
        case(4): return BACK_BUMPER;
            break;
        case(5): return BACK_RIGHT;
            break;
        default: SWERRINT(value);
    };
    return FRONT_LEFT;
}

enum averages
{
    TIME_THROUGH_INTERSECTION, //0
    TIME_IN_INTERSECTION,      //1
    TIME_AT_MAX_SPEED,         //2
    TIME_STOPPED,              //3
    TIME_BETWEEN_SPAWNS,       //4
    FUEL_CONSUMPTION,          //5
    CO2_EMISSIONS,             //6
    TOTAL_AVERAGES,            //7
};

const std::string AVERAGE_STR[]
{
    "Time Through Intersection",
    "Time In Intersection",
    "Time At Max Speed",
    "Time Stopped",
    "Time Between Spawns",
    "Fuel Consumption",
    "CO2 EMISSIONS",
    "Total Averages",
};

const std::string AVERAGE_UNITS_STR[]
{
    "[s]",
    "[s]",
    "[s]",
    "[s]",
    "[s]",
    "[mL]",
    "[kg]",
    "",

};

enum Grade
{
    TERRIBLE,
    BAD,
    MEDIOCRE,
    STANDARD,
    GOOD,
    GREAT,
    EXCELLENT,
};

const std::string GRADE_STR[]
{
    "TERRIBLE",
    "BAD",
    "MEDIOCRE",
    "STANDARD",
    "GOOD",
    "GREAT",
    "EXCELLENT",
};

enum severity
{
    LOW,        //0
    MINOR,      //1
    MEDIUM,     //2
    MODERATE,   //3
    HIGH,       //4
    FATAL,      //5
};

const std::string SEVERITY_STR[]
{
    "LOW",
    "MINOR",
    "MEDIUM",
    "MODERATE",
    "HIGH",
    "FATAL",
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
const VehicleParameters vehicle_params;

static Grade fuelConsumptionGrade(float fuel_consumption_)
{
    if(fuel_consumption_ < 0)
    {
        SWERRFLOAT(fuel_consumption_);
        return fuelConsumptionGrade(-1 * fuel_consumption_);
    }
    else if(fuel_consumption_ < 20)
    {
        return EXCELLENT;
    } 
    else if(fuel_consumption_ < 22)
    {
        return GREAT;
    }
    else if (fuel_consumption_ < 24)
    {
        return GOOD;
    }
    else if (fuel_consumption_ < 26)
    {
        return STANDARD;
    } 
    else if (fuel_consumption_ < 28)
    {
        return MEDIOCRE;
    }
    else if (fuel_consumption_ < 30)
    {
        return BAD;
    }
    else //fuel consumption >= 32
    {
        return TERRIBLE;
    }
}

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

//these may be necessary in replacing maxComponent (abs() isn't working)
static bool findComponent(float first_point_[2], float second_point_[2])
{
    float delta_x, delta_y;

    delta_x = first_point_[x] - second_point_[x];
    if (delta_x < 0)
    {
        delta_x *= -1;
    }

    delta_y = first_point_[y] - second_point_[y];
    if (delta_y < 0)
    {
        delta_y *= -1;
    }

    return delta_y > delta_x;
}

static bool findComponent(float first_value_, float second_value_)
{
    if(first_value_ < 0)
    {
        first_value_ *= -1;
    }
    if (second_value_ < 0)
    {
        second_value_ *= -1;
    }
    return second_value_ > first_value_;
}

static float neededAcceleration(float current_velocity_, float distance_)
{
    return (-1 * current_velocity_ * current_velocity_) / (2 * distance_);
}

static float neededDistance(float current_velocity_, float acceleration_)
{
    return (-1 * current_velocity_ * current_velocity_) / (2 * acceleration_);
}

static float distanceBetween(float first_point_[2], float second_point_[2])
{
    float delta_x = second_point_[x] - first_point_[x];
    float delta_y = second_point_[y] - first_point_[y];

    return sqrt((delta_x * delta_x) + (delta_y * delta_y));
}

//magnitude of a vector
constexpr float MAGNITUDE(float x_value, float y_value)
{
    return sqrt( (x_value * x_value ) + ( y_value * y_value ) );
}

constexpr float FROM_MAGNITUDE(float h_value, float x_value)
{
    return sqrt( (pow(h_value, 2)) - (pow(x_value, 2)) );
}

constexpr bool XOR(bool a, bool b)
{
    return !(a) != !(b); 
}

template <typename type>
type max(type a, type b)
{
    return (abs(a) > abs(b)) ? a : b;
}

template <typename type>
type min(type a, type b)
{
    return (abs(a) < abs(b)) ? a : b;
}

template <typename type>
bool maxComponent(type x_component, type y_component)
{
    return (abs(x_component) >= abs(y_component)) ? x : y;
}

template <typename type>
bool minComponent(type x_component, type y_component)
{
    return (abs(x_component) >= abs(y_component)) ? y : x;
}

template <typename type>
bool isPositive(type value)
{
    return value >= 0;
}

template <typename type>
float stoppingDistance(type current_velocity_, type acceleration)
{
    return (-1 * pow(current_velocity_, 2)) / (2 * acceleration);
}

template <typename type>
float requiredAcceleration(type current_velocity_, type distance)
{
    return (-1 * pow(current_velocity_, 2)) / (2 * distance);
}
#endif