#pragma once
#include "./Road.h"

/*
*   Name: NorthRoad
*
*   Description: Overrides base Road functions to create specialized road
*
*   Type: Derived Class
*
*   Inherits: Road
*/
class NorthRoad : public Road
{
public:
    NorthRoad();
    direction correspondingExit(path path_);
private:
};