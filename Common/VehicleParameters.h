#pragma once

#include "./CommonTypes.h"

class VehicleParameters
{
public:
    VehicleParameters();
    uint8 vehicle_length = 6;
    uint8 vehicle_width = 2;
    bool print_velocieties;
    bool print_exterior_coordinates;
private:
};