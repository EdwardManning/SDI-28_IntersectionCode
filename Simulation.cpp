#include "Simulation.h"

//Function definitions for Simulation.h

/*
*   Name: Simulation
*
*   Description: Assigns values to private variables and starts the simulation.
*
*   Input: N/A
*
*   Output: N/A
*
*/
Simulation::Simulation()
{
    car = new Car(0, STRAIGHT, my_intersection.getRoad(EAST)->getLane(6), NORMAL);
    elapsed_time = 0;
    run();
}

/*
*   Name: run
*
*   Description: This is the function that is responsible for running the simulation.
*                It uses a loop to control time and completes the actions for each vehicle
*                at each loop iteration (time step).
*
*   Input: N/A
*
*   Output: N/A
*
*/
void Simulation::run()
{
    debugIntersection();
    while(completionCheck())
    {
        if (car->vehicleType() == CAR)
        {
            driverPerformActions(car);
        }
        else
        {
            //will eventually be used with vehiclePerformActions
            //for autonomous vehicles
            SWERRINT(car->vehicleType());
        }
        elapsed_time += simulation_params.time_step;
    }
    std::cout << car->timeInIntersection() << " " << car->currentPosition()[x] << " " << car->currentPosition()[y] << std::endl;
}


/*
*   Name: completionCheck
*
*   Description: Checks if the simulation is complete.
*
*   Input: N/A
*
*   Output: True if simulation is complete, false otherwise.
*
*/
bool Simulation::completionCheck()
{
    return car->currentPosition()[x] > 0;
}

/*
*   Name: driverPerformActions
*
*   Description: Acts as the brain of the driver. Used to
*                determine what actions to do at what time.
*
*   Input: vehicle_ -> Pointer to the vehicle in question.
*
*   Output: N/A
*
*/
void Simulation::driverPerformActions(Vehicle* vehicle_)
{
    std::cout << (int)vehicle_->currentState() << " " << vehicle_->currentPosition()[x] << " " << vehicle_->currentPosition()[y] << std::endl;
    if(passedStopLine(vehicle_) && 
       !(vehicle_->currentState() & IN_INTERSECTION) &&
       !(vehicle_->currentState() & THROUGH_INTERSECTION))
    {
        vehicle_->changeState(IN_INTERSECTION, ADD);
    }

    if(vehicle_->currentState() & IN_INTERSECTION)
    {
        if(passedExitStartLine(vehicle_))
        {
            vehicle_->changeState(IN_INTERSECTION, REMOVE);
            setLane(vehicle_);
            vehicle_->changeState(THROUGH_INTERSECTION, ADD);
        }
    }
    
    if(!(vehicle_->currentState() & CORRECT_LANE))
    {
        path lane_change_direction = changeLaneDirection(vehicle_);
        if(vehicle_->currentState() & CHANGING_LANES)
        {
            if (overCenterLine(vehicle_, lane_change_direction))
            {
                int8 lane_change_modifier = lane_change_direction == LEFT ? (-1) : 1;
                uint8 new_lane = lane_change_modifier + vehicle_->laneNumber();
                vehicle_->setLane(new_lane);
                if(vehicle_->correctLane(my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())))
                {
                    vehicle_->changeState(CHANGING_LANES, REMOVE);
                }
                vehicle_->stopLaneChange();
            }
        }
        else
        {
            vehicle_->changeState(CHANGING_LANES, ADD);
            vehicle_->changeLane(lane_change_direction);
        }
    }
    vehicle_->drive();
}

/*
*   Name: vehiclePerformActions
*
*   Description: Acts as the self-driving software. Used to
*                determine what actions to do at what time.
*
*   Input: atonomous_vehicle -> Pointer to the vehicle in question.
*
*   Output: N/A
*
*/
void Simulation::vehiclePerformActions(Vehicle* autonomous_vehicle_)
{

}

bool Simulation::passedStopLine(Vehicle* vehicle_)
{
    switch(vehicle_->vehicleDirection())
    {
        case(NORTH): return vehicle_->currentPosition()[y] > my_intersection.getRoad(NORTH)->endingPosition();
            break;
        case(SOUTH): return vehicle_->currentPosition()[y] < my_intersection.getRoad(SOUTH)->endingPosition();
            break;
        case(EAST): return vehicle_->currentPosition()[x] < my_intersection.getRoad(EAST)->endingPosition();
            break;
        case(WEST): return vehicle_->currentPosition()[x] > my_intersection.getRoad(WEST)->endingPosition();
            break;
        default: SWERRINT(vehicle_->vehicleDirection());
    }
    return false;
}

/*
*   Name: passedExitStartLine
*
*   Description: Determines if the vehicle is passed the start line 
*                of the exit lane. Used as definition of when a vehicle
*                is out of an the intersection.
*
*   Input: vehicle_ -> Pointer to the vehicle in question.
*
*   Output: True if the vehicle is out of the intersection otherwise false.
*
*/
bool Simulation::passedExitStartLine(Vehicle* vehicle_)
{
    switch(my_intersection.getRoad(vehicle_->vehicleDirection())->correspondingExit(vehicle_->vehiclePath()))
    {
        case(NORTH): return vehicle_->currentPosition()[y] <= my_intersection.getRoad(NORTH)->endingPosition();
            break;
        case(SOUTH): return vehicle_->currentPosition()[y] >= my_intersection.getRoad(SOUTH)->endingPosition();
            break;
        case(EAST): return vehicle_->currentPosition()[x] >= my_intersection.getRoad(EAST)->endingPosition();
            break;
        case(WEST): return vehicle_->currentPosition()[x] <= my_intersection.getRoad(WEST)->endingPosition();
            break;
        default: SWERRINT(my_intersection.getRoad(vehicle_->vehicleDirection())->correspondingExit(vehicle_->vehiclePath()));
    }
    return false;
}

/*
*   Name: setLane
*
*   Description: Sets the lane of the vehicle. Used after vehicle
*                exits the intersection and is now in the exit lane.
*
*   Input: vehicle_ -> Pointer to the vehicle in question.
*
*   Output: N/A
*
*/
void Simulation::setLane(Vehicle* vehicle_)
{
    Road* road = my_intersection.getRoad(my_intersection.getRoad(vehicle_->vehicleDirection())->correspondingExit(vehicle_->vehiclePath()));
    switch(my_intersection.getRoad(vehicle_->vehicleDirection())->correspondingExit(vehicle_->vehiclePath()))
    {
        case(NORTH): vehicle_->setLane(closestLane(vehicle_->currentPosition()[x], road));
            break;
        case(SOUTH): vehicle_->setLane(closestLane(vehicle_->currentPosition()[x], road));
            break;
        case(EAST): vehicle_->setLane(closestLane(vehicle_->currentPosition()[y], road));
            break;
        case(WEST): vehicle_->setLane(closestLane(vehicle_->currentPosition()[y], road));
            break;
        default: SWERRINT(my_intersection.getRoad(vehicle_->vehicleDirection())->correspondingExit(vehicle_->vehiclePath()));
    }
}

/*
*   Name: closestLane
*
*   Description: Determines the closest lane to the vehicle.
*
*   Input: checked_position_ -> The position coordinate of the
*                               vehicle being checked against the lane.
*          road_             -> The road holding the lanes being checked.
*
*   Output: The lane number of the closest lane.
*
*/
uint8 Simulation::closestLane(float checked_position_, Road* road_)
{
    uint8 closest_road = -1;
    float closest_distance = intersection_params.frame_length; 
    if (road_->roadDirection() == NORTH || road_->roadDirection() == SOUTH)
    {
        for(uint8 i = 0; i < intersection_params.ns_number_of_exits; i++)
        {
            if (abs(checked_position_ - road_->getLane(i)->centerLine()) < closest_distance)
            {
                closest_distance = abs(checked_position_ - road_->getLane(i)->centerLine());
                closest_road = i;
            }
        }
    }
    else if (road_->roadDirection() == EAST || road_->roadDirection() == WEST)
    {
        for(uint8 i = 0; i < intersection_params.ew_number_of_exits; i++)
        {
            if (abs(checked_position_ - road_->getLane(i)->centerLine()) < closest_distance)
            {
                closest_distance = abs(checked_position_ - road_->getLane(i)->centerLine());
                closest_road = i;
            }
        }
    }
    else
    {
        SWERRINT(road_->roadDirection());
    }

    if(closest_road == -1)
    {
        SWERRFLOAT(closest_distance);
        return 0;
    }

    if (closest_distance > 0.6 * intersection_params.lane_width)
    {
        SWERRFLOAT(closest_distance);
    }

    return closest_road;
}

/*
*   Name: debugIntersection
*
*   Description: Displays the name and center line of each lane and the
*                name of each road.
*
*   Input: N/A
*
*   Output: N/A
*
*/
void Simulation::debugIntersection()
{
    for(uint8 i = 0; i < TOTAL_DIRECTIONS; i++)
    {
        std::cout << my_intersection.getRoad(i)->name() << std::endl;
        for(uint8 j = 0; j < my_intersection.getRoad(i)->totalLanes(); j++)
        {
            std::cout << my_intersection.getRoad(i)->getLane(j)->name() << " " << my_intersection.getRoad(i)->getLane(j)->centerLine() << std::endl;
        }
    }
}

/*
*   Name: changeLaneDirection
*
*   Description: Determines the direction the vehicle needs to
*                change lane in.
*
*   Input: vehicle_ -> Pointer to the vehicle in question.
*
*   Output: The direction of the lane change (Left or Right).
*
*/
path Simulation::changeLaneDirection(Vehicle* vehicle_)
{
    path current_lane_path = my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->lanePath();

    switch(vehicle_->vehiclePath())
    {
        case(LEFT):
        {
            return RIGHT;
        }
            break;
        case(RIGHT):
        {
            return LEFT;
        }
            break;
        case(STRAIGHT):
        {
            return current_lane_path == LEFT || current_lane_path == STRAIGHT_LEFT ? RIGHT : LEFT;
        }
            break;
        default:SWERRINT(vehicle_->vehiclePath());
    }
    return LEFT;
}

/*
*   Name: overCenterLine
*
*   Description: Determines if the vehicle has passed over the center
*                line of the next lane. If so, the lane change is complete.
*
*   Input: vehicle_               -> Pointer to the vehicle in question.
*          lane_change_direction_ -> The direction of the current lane change.
*
*   Output: True if it's is over the center line, false otherwise.
*
*/
bool Simulation::overCenterLine(Vehicle* vehicle_, path lane_change_direction_)
{
    int8 lane_change_modifier = lane_change_direction_ == LEFT ? -1 : 1;
    uint8 current_lane = vehicle_->laneNumber();
    uint8 next_lane_number = current_lane + lane_change_modifier;
    Lane* next_lane = my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(next_lane_number);
    switch(vehicle_->vehicleDirection())
    {
        case(NORTH):
        {
            if(lane_change_direction_ == LEFT)
            {
                return (vehicle_->currentPosition()[x] >= next_lane->centerLine());
            }
            else
            {
                return (vehicle_->currentPosition()[x] <= next_lane->centerLine());
            }
        }
            break;
        case(SOUTH):
        {
            if(lane_change_direction_ == LEFT)
            {
                return (vehicle_->currentPosition()[x] <= next_lane->centerLine());
            }
            else
            {
                return (vehicle_->currentPosition()[x] >= next_lane->centerLine());
            }
        }
            break;
        case(EAST):
        {
            if(lane_change_direction_ == LEFT)
            {
                return (vehicle_->currentPosition()[y] >= next_lane->centerLine());
            }
            else
            {
                return (vehicle_->currentPosition()[y] <= next_lane->centerLine());
            }
        }
            break;
        case(WEST):
        {
            if(lane_change_direction_ == LEFT)
            {
                return (vehicle_->currentPosition()[y] <= next_lane->centerLine());
            }
            else
            {
                return (vehicle_->currentPosition()[y] >= next_lane->centerLine());
            }
        }
            break;
        default: SWERRINT(vehicle_->vehicleDirection());
    }
    return true;
}