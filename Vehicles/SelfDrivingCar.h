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
class SelfDrivingCar : public Vehicle
{
public:
    SelfDrivingCar(uint32 number_, path path_, Lane* lane_);
    ~SelfDrivingCar();
protected:
    void consumeFuel();
private:
};