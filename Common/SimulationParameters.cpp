#include "./SimulationParameters.h"
#include "SimulationParameters.h"

//Function definitions for SimulationParameters.h

/*
*   Name: SimulationParameters
*
*   Description: Does Nothing
*
*   Input: N/As
*
*   Output: N/A
*
*/
SimulationParameters::SimulationParameters()
{
    print_simulation_events = false;
    print_vehicle_info = false;
    print_debug_info = false;
    print_debug_acceleration = false;
    print_results_for_python = true;
    if((print_debug_acceleration && !print_debug_info) || (number_of_vehicles > 100))
    {
        print_debug_acceleration = false;
    }
    input.open("./Input/SimulationParamsInput.txt");
    if(input.is_open())
    {
        uint32 counter = 0;
        while(input && counter < 3)
        {
            switch(counter)
            {
                case(0):
                {
                    std::string placeholder;
                    std::getline(input, placeholder);
                    spawn_density = char(std::stoi(placeholder));
                }
                    break;
                case(1): 
                {
                    std::string placeholder;
                    std::getline(input, placeholder);
                    self_driving_vehicle_probability = char(std::stoi(placeholder));
                }
                    break;
                case(2):
                {
                    std::string placeholder;
                    std::getline(input, placeholder);
                    control_system = bool(std::stoi(placeholder));
                }
                    break;
                default:
                {
                    throw;
                }
            }
            counter++;
        }
    }
    else
    {
        throw;
    }
    if(input.is_open())
    {
        input.close();
    }
}

float ns_squeeze_ns(float time)
{
    return 25 + 20 * (std::sin((PI/200)*time) * std::sin((PI/200)*time));
}

float ns_squeeze_ew(float time)
{
    return 5 + 20 * (std::cos((PI/200)*time) * std::cos((PI/200)*time));
}

float ew_squeeze_ns(float time)
{
    return 5 + 20 * (std::cos((PI/200)*time) * std::cos((PI/200)*time));
}

float ew_squeeze_ew(float time)
{
    return 25 + 20 * (std::sin((PI/200)*time) * std::sin((PI/200)*time));
}

float north_constant(float time)
{
    return 25;
}

float south_constant(float time)
{
    return 25;
}

float east_constant(float time)
{
    return 25;
}

float west_constant(float time)
{
    return 25;
}
