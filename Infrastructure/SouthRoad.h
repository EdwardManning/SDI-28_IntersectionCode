#pragma once
#include "./Road.h"

/*
*   Name: SouthRoad
*
*   Description: Overrides base Road functions to create specialized road
*
*   Type: Derived Class
*
*   Inherits: Road
*/
class SouthRoad : public Road
{
public:
    SouthRoad();
    direction correspondingExit(path path_);
private:
};