#pragma once
#include <fstream>
#include <string>
#include <cmath>
#include "./CommonTypes.h"

#define PI  3.1415926535

static float ns_squeeze_ns(float time);
static float ns_squeeze_ew(float time);

static float ew_squeeze_ns(float time);
static float ew_squeeze_ew(float time);

static float north_constant(float time);
static float south_constant(float time);
static float east_constant(float time);
static float west_constant(float time);

/*
*   Name: SimulationParameters
*
*   Description: Stores and defines the parameters of the simulation.
*
*   Type: Regular Class
*/
class SimulationParameters
{
public:
    SimulationParameters();
    bool control_system;
    float time_step = 0.01;
    uint32 number_of_vehicles = 100;
    uint8 spawn_density; //spawn chance at 5 seconds since the previous vehicle was spawned
    uint8 self_driving_vehicle_probability;
    probability north_spawn_probability = north_constant;
    probability south_spawn_probability = south_constant;
    probability east_spawn_probability = east_constant;
    probability west_spawn_probability = west_constant;
    bool print_simulation_events;
    bool print_vehicle_info;
    bool print_debug_info;
    bool print_debug_acceleration;
    bool print_results_for_python;
protected:
private:
    //std::string input_file_name = "SimulationParamsInput.txt";
    std::ifstream input;
};