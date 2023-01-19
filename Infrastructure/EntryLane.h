#pragma once
#include "./Lane.h"

/*
*   Name: EntryLane
*
*   Description: Overrides base Lane functions to create specialized entry lane
*
*   Type: Derived Class
*
*   Inherits: Lane
*/
class EntryLane : public Lane
{
public:
    EntryLane(uint8 number_, direction direction_, path path_);
private:
};