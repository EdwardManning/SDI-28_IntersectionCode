#pragma once
#include "Vehicle.h"

/*
*   Name: Car
*
*   Description: Overrides base vehicles functions and variables to 
*                create specialized car vehicle.
*
*   Type: Derived Class
*
*   Inherits: Vehicle
*
*/
class Car : public Vehicle
{
public:
    Car(uint16 number_, path path_, Lane* lane_, DriverType driver_type_);
private:
};