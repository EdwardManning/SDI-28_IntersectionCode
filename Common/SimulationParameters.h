#pragma once
#include "./CommonTypes.h"

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
    float time_step = 0.01;
    uint32 number_of_vehicles = 10;
    uint8 spawn_density = 1; //spawn chance at 5 seconds since the previous vehicle was spawned
    uint8 self_driving_vehicle_probability = 25;
    uint8 north_spawn_probability = 25;
    uint8 south_spawn_probability = 25;
    uint8 east_spawn_probability = 25;
    uint8 west_spawn_probability = 25;
    bool print_simulation_events;
    bool print_vehicle_info;
    bool print_debug_info;
protected:
private:
};