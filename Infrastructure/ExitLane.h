#pragma once
#include "./Lane.h"

/*
*   Name: ExitLane
*
*   Description: Overrides base Lane functions to create specialized entry lane
*
*   Type: Derived Class
*
*   Inherits: Lane
*/
class ExitLane : public Lane
{
public:
    ExitLane(uint8 number_, direction direction_, path path_);
private:
};