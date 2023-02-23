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
    uint32 number_of_vehicles = 5;
    bool print_simulation_events;
    bool print_vehicle_info;
protected:
private:
};