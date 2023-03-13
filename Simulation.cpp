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
    events.open("./Output/Events.txt");
    if(!simulation_params.print_simulation_events)
    {
        events << "No Event Printing" << std::endl;
    }
    my_vehiclesMade = 0;
    for(uint8 i = 0; i < TOTAL_AVERAGES; i++)
    {
        averages[i] = 0;
    }
    vehicle_list = new Vehicle*[simulation_params.number_of_vehicles];
    generateVehicle(my_vehiclesMade);
    my_vehiclesMade++;
    my_spawnTimer = 0;
    light_change_occured = false;
    elapsed_time = 0;
    run();
}

/*
*   Name: ~Simulation
*
*   Description: Cleans up parts that must be cleaned up and prints results.
*
*   Input: N/A
*
*   Output: N/A
*
*/
Simulation::~Simulation()
{
    events.close();
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
    //debugIntersection();
    while(!completionCheck() && (elapsed_time < 2000))
    {   
        for(uint32 i = 0; i < active_vehicles.size(); i++)
        {
            if (active_vehicles[i]->vehicleType() == CAR)
            {
                if(!(active_vehicles[i]->isCompleted()))
                {
                    driverPerformActions(active_vehicles[i]);
                }
            }
            else
            {
                //will eventually be used with vehiclePerformActions
                //for autonomous vehicles
                SWERRINT(active_vehicles[i]->vehicleType());
            }
        }
        if(my_intersection.trafficLight()->makeStep())
        {
            light_change_occured = true;
            if(simulation_params.print_simulation_events)
            {
                printTrafficLightStateChange(my_intersection.trafficLight());
            }
        }
        else
        {
            light_change_occured = false;
        }
        
        elapsed_time += simulation_params.time_step;
        if (my_vehiclesMade < simulation_params.number_of_vehicles)
        {
            if(spawnVehicle())
            {
                generateVehicle(my_vehiclesMade);
                my_vehiclesMade++;
                averages[TIME_BETWEEN_SPAWNS] += my_spawnTimer;
                my_spawnTimer = 0;
            }
            else
            {
                my_spawnTimer += simulation_params.time_step;
            }
        }

    }
    if(elapsed_time >= 2000)
    {
        std::cout << my_vehiclesMade << std::endl;
    }
    std::cout << elapsed_time << std::endl;
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
    if(collisionAnalysis()) //collision happened
    {
        return true;
    }
    bool return_value = true;
    for(uint32 i = 0; i < my_vehiclesMade; i++)
    {
        if (!vehicleCompleted(vehicle_list[i]))
        {
            return_value = false;
        }
    }
    if(my_vehiclesMade < simulation_params.number_of_vehicles)
    {
        return false;
    }
    else if(return_value)
    {
        printResults();
    }
    return return_value;
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
    if(passedStopLine(vehicle_) && 
       !(vehicle_->currentState() & IN_INTERSECTION) &&
       !(vehicle_->currentState() & THROUGH_INTERSECTION))
    {
        changeState(vehicle_, IN_INTERSECTION, ADD);
        //if it is not already in the intersection list
        if(!(my_intersection.inIntersection(vehicle_->number())))
        {
            my_intersection.addToIntersection(vehicle_->number());
        }
        else
        {
            SWERRINT(vehicle_->number()<<8 + vehicle_->currentState());
        }
        //if it is in the lane
        if(my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->inLane(vehicle_->number()))
        {
            my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->removeFromLane(vehicle_->number());
        }
        else
        {
            std::cout << vehicle_->number() << " " << (int)vehicle_->currentState() << " " << (int)vehicle_->laneNumber() << std::endl;
            SWERRINT(vehicle_->number()<<8 + vehicle_->currentState());
        }
        //if it is in the road
        if(my_intersection.getRoad(vehicle_->vehicleDirection())->inRoad(vehicle_->number()))
        {
            my_intersection.getRoad(vehicle_->vehicleDirection())->removeFromRoad(vehicle_->number());
        }
        else
        {
            SWERRINT(vehicle_->number()<<8 + vehicle_->currentState());
        }

        if (vehicle_->vehiclePath() != STRAIGHT)
        {
            changeState(vehicle_, TURNING, ADD);
        }
    }

    if(vehicle_->currentState() & IN_INTERSECTION)
    {
        if(passedExitStartLine(vehicle_))
        {
            changeState(vehicle_, IN_INTERSECTION, REMOVE);
            //if in intersection
            if(my_intersection.inIntersection(vehicle_->number()))
            {
                my_intersection.removeFromIntersection(vehicle_->number());
            }
            else
            {
                SWERRINT(vehicle_->currentState());
            }
            setLane(vehicle_);
            changeState(vehicle_, THROUGH_INTERSECTION, ADD);
            if(vehicle_->currentState() & TURNING)
            {
                direction exit_direction = my_intersection.getRoad(vehicle_->vehicleDirection())->correspondingExit(vehicle_->vehiclePath());
                changeState(vehicle_, TURNING, REMOVE);
                vehicle_->stopTurn(my_intersection.getRoad(exit_direction)->getLane(vehicle_->laneNumber()));
            }
        }
        else if (vehicle_->currentState() & TURNING)
        {
            vehicle_->turn();
        }
    }

    accelerate(vehicle_);
    
    if(!(vehicle_->currentState() & CORRECT_LANE))
    {
        path lane_change_direction = changeLaneDirection(vehicle_);
        if(!vehicle_->blinker(lane_change_direction))
        {
            vehicle_->toggleBlinker(lane_change_direction, ON);
        }
        if(vehicle_->currentState() & CHANGING_LANES)
        {
            if (overCenterLine(vehicle_, lane_change_direction))
            {
                int8 lane_change_modifier = lane_change_direction == LEFT ? (-1) : 1;
                uint8 new_lane = lane_change_modifier + vehicle_->laneNumber();
                if (simulation_params.print_simulation_events)
                {
                    printLaneChange(vehicle_, new_lane);
                }
                if(my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->inLane(vehicle_->number()))
                {
                    my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->removeFromLane(vehicle_->number());
                }
                vehicle_->setLane(new_lane);
                if(!my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->inLane(vehicle_->number()))
                {
                    my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->addToLane(vehicle_->number());
                }
                if(vehicle_->correctLane(my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())))
                {
                    vehicle_->toggleBlinker(lane_change_direction, OFF);
                    changeState(vehicle_, CORRECT_LANE, ADD);
                    changeState(vehicle_, CHANGING_LANES, REMOVE);
                    vehicle_->stopLaneChange();
                }
            }
        }
        else
        {
            if(!laneChangeDecelerationRequired(vehicle_))
            {
                changeState(vehicle_, CHANGING_LANES, ADD);
                vehicle_->changeLane(lane_change_direction);
            }
        }
    }
    if ((vehicle_->currentState() & ACCELERATING) | (vehicle_->currentState() & DECELERATING))
    {
        if(vehicle_->accelerate())
        {
            if (vehicle_->currentState() & DECELERATING)
            {
                changeState(vehicle_, DECELERATING, REMOVE);
                vehicle_->toggleBrakeLights(OFF);
                if(MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]) == 0)
                {
                    changeState(vehicle_, DRIVING, REMOVE);
                }
            }
            else if (vehicle_->currentState() & ACCELERATING)
            {
                changeState(vehicle_, ACCELERATING, REMOVE);
            }
            else
            {
                SWERRINT(vehicle_->currentState());
            }
        }
    }
    if((MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]) < 0.005) &&
       (vehicle_->currentState() & DECELERATING))
    {
        changeState(vehicle_, DRIVING, REMOVE);
        changeState(vehicle_, DECELERATING, REMOVE);
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
*                NOTE: THIS IS NOT USED IN LANE CHANGES.
*
*   Input: vehicle_ -> Pointer to the vehicle in question.
*
*   Output: N/A
*
*/
void Simulation::setLane(Vehicle* vehicle_)
{
    if(my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->inLane(vehicle_->number()))
    {
       my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->removeFromLane(vehicle_->number()); 
    }
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
    if(!my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->inLane(vehicle_->number()))
    {
        my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->addToLane(vehicle_->number());
    }
    if(!my_intersection.getRoad(road->roadDirection())->inRoad(vehicle_->number()))
    {
       my_intersection.getRoad(road->roadDirection())->addToRoad(vehicle_->number()); 
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
            std::cout << my_intersection.getRoad(i)->getLane(j)->name() << " " << my_intersection.getRoad(i)->getLane(j)->centerLine() << " ";
            std::cout << my_intersection.getRoad(i)->getLane(j)->startingPosition()[x] << " " << my_intersection.getRoad(i)->getLane(j)->startingPosition()[y] << " ";
            std::cout << my_intersection.getRoad(i)->getLane(j)->endingPosition()[x] << " " << my_intersection.getRoad(i)->getLane(j)->endingPosition()[x] << std::endl;
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
            return LEFT;
        }
            break;
        case(RIGHT):
        {
            return RIGHT;
        }
            break;
        case(STRAIGHT):
        {
            return current_lane_path == LEFT ? RIGHT : LEFT;
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

bool Simulation::shoulderCheck(Vehicle* vehicle_, path lane_change_direction_)
{
    int8 lane_change_modifier = lane_change_direction_ == LEFT ? -1 : 1;
    uint8 current_lane = vehicle_->laneNumber();
    uint8 next_lane_number = current_lane + lane_change_modifier;

    if(my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(next_lane_number)->numberOfVehicles() == 0)
    {
        return true;
    }
    else
    {
        for(uint32 i = 0; i < my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(next_lane_number)->numberOfVehicles(); i++)
        {
            if (vehicle_->checkImportantPosition(vehicle_list[my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(next_lane_number)->vehicleAtIndex(i)]))
            {
                return false;
            }
        }
        return true;
    }
}

/*
*   Name: changeState
*
*   Description: Responsible for sending state change to vehicle and printing event.
*
*   Input: vehicle_ -> The vehicle in question.
*          state_   -> The state being sent to the vehicle.
*          adding_  -> Boolean stating if state is being added or removed.
*
*   Output: N/A
*
*/
void Simulation::changeState(Vehicle* vehicle_, state state_, const bool adding_)
{
    if (simulation_params.print_simulation_events)
    {
        uint8 current_state = vehicle_->currentState();
        vehicle_->changeState(state_, adding_);
        events << elapsed_time << " Vehicle " << vehicle_->number() << ": changed from state " << (int)current_state << " to " << (int)vehicle_->currentState() << std::endl;
    }
    else
    {
        vehicle_->changeState(state_, adding_);
    }
}

/*
*   Name: vehicleCompleted
*
*   Description: Determines if a vehicle is through the intersection or not.
*
*   Input: vehicle_ -> The vehicle in question.
*
*   Output: True if the vehicle has completed the intersection, false otherwise.
*
*/
bool Simulation::vehicleCompleted(Vehicle* vehicle_)
{
    if (vehicle_->isCompleted())
    {
        return true;
    }
    bool return_value = false;
    if (vehicle_->currentState() & THROUGH_INTERSECTION) //cannot be completed without going through the intersection
    {
        bool max_component = maxComponent<float>(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]);
        if (vehicle_->currentVelocity()[max_component] > 0) //heading in a direction with a positive velocity vector
        {
            if(max_component) //if in y direction
            {
                if (vehicle_->currentPosition()[max_component] > intersection_params.frame_length) //if it's off the screen in the y direction
                {
                    if(simulation_params.print_simulation_events)
                    {
                        printCompletion(vehicle_);
                    }
                    vehicle_->completed();
                    return_value = true;
                }
            }
            else
            {
                if (vehicle_->currentPosition()[max_component] > intersection_params.frame_width) //if it's off the screen in the x direction
                {
                    if(simulation_params.print_simulation_events)
                    {
                        printCompletion(vehicle_);
                    }
                    vehicle_->completed();
                    return_value = true;
                }
            }
        }
        else //negative velocity vector (must go off at 0 in one of the two directions)
        {
            if (vehicle_->currentPosition()[max_component] < 0) //current position is off the frame
            {
                if(simulation_params.print_simulation_events)
                {
                    printCompletion(vehicle_);
                }
                vehicle_->completed();
                return_value = true;
            }
        }
    }
    else
    {
        return false; //is not through intersection
    }
    if(return_value)
    {
        if(isActive(vehicle_))
        {
            removeFromActiveVehicles(vehicle_);
        }
        direction exit_direction = my_intersection.getRoad(vehicle_->vehicleDirection())->correspondingExit(vehicle_->vehiclePath());
        if (my_intersection.getRoad(exit_direction)->getLane(vehicle_->laneNumber())->inLane(vehicle_->number()))
        {
            my_intersection.getRoad(exit_direction)->getLane(vehicle_->laneNumber())->removeFromLane(vehicle_->number());
        }
        if(my_intersection.getRoad(exit_direction)->inRoad(vehicle_->number()))
        {
            my_intersection.getRoad(exit_direction)->removeFromRoad(vehicle_->number());
        }
        if(!vehicle_->isCompleted())
        {
            vehicle_->completed();
            if(simulation_params.print_simulation_events)
            {
                printCompletion(vehicle_);
            }
        }
    }
    return return_value;
}

//acceleration related code
void Simulation::accelerate(Vehicle* vehicle_)
{
    float proximity_deceleration_distance_required = determineCloseProximityDecelerationDistance(vehicle_);
    float light_deceleration_distance_required = determineLightBasedDecelerationDistance(vehicle_);

    if(proximity_deceleration_distance_required < 0)
    {
        if(light_deceleration_distance_required == 0)
        {  
            SWERRFLOAT(proximity_deceleration_distance_required);
            proximity_deceleration_distance_required *= -1;
        }
        else
        {
            proximity_deceleration_distance_required = 0;
        }
    }

    if(light_deceleration_distance_required < 0)
    {
        if(proximity_deceleration_distance_required == 0)
        {
            SWERRFLOAT(light_deceleration_distance_required);
            light_deceleration_distance_required *= -1;
        }
        else
        {
            light_deceleration_distance_required = 0;
        }
    }

    // if(proximity_deceleration_distance_required < 0.05)
    // {
    //     proximity_deceleration_distance_required = 0;
    // }
    // if(light_deceleration_distance_required < 0.05)
    // {
    //     light_deceleration_distance_required = 0;
    // }

    // if(vehicle_->currentState() & DRIVING && vehicle_->currentState() & ACCELERATING)
    // {
    //     if (proximity_deceleration_distance_required == 0 && light_deceleration_distance_required == 0)
    //     {
    //         startAcceleration(vehicle_, vehicle_->maxSpeed());
    //         return;
    //     }
    // }

    if(vehicle_->currentState() & IN_INTERSECTION &&
       (my_intersection.trafficLight()->currentEvent() == EW_MUTUAL_RED ||
       my_intersection.trafficLight()->currentEvent() == NS_MUTUAL_RED))
    {
        startAcceleration(vehicle_, vehicle_->maxSpeed());
        return;
    }

    //std::cout << proximity_deceleration_distance_required << " " << light_deceleration_distance_required << std::endl;

    //we are going to need to adjust this code for if the vehicle is stopped
    if(vehicle_->currentState() & DRIVING)
    {
        if(proximity_deceleration_distance_required != 0)
        {
            if(proximity_deceleration_distance_required > light_deceleration_distance_required && light_deceleration_distance_required != 0)
            {
                //if light deceleration distance is lower than the proximity deceleration distance and non zero
                startDeceleration(vehicle_, STOP, light_deceleration_distance_required);
            }
            else
            {
                //proximity deceleration distance is not zero and light deceleration distance is either zero or greater than it
                startDeceleration(vehicle_, STOP, proximity_deceleration_distance_required);
            }
        }
        else if (light_deceleration_distance_required != 0)
        {
            //if proximity deceleration distance is zero and light deceleration is non zero then light deceleration is  
            //automatically the least non-zero deceleration distance
            startDeceleration(vehicle_, STOP, light_deceleration_distance_required);
        }
        else //both are zero
        {
            //we can speed up
            startAcceleration(vehicle_, vehicle_->maxSpeed());
        }
    }
    else
    {
        if(!(vehicle_->currentState() & IN_INTERSECTION) && !(vehicle_->currentState() & THROUGH_INTERSECTION))
        {
            bool dot = findComponent(vehicle_->exteriorPosition(FRONT_BUMPER), vehicle_->exteriorPosition(BACK_BUMPER));
            if(!light_change_occured)
            {
                if(abs(vehicle_->unitVector()[dot] * (vehicle_->exteriorPosition(FRONT_BUMPER)[dot] - vehicle_->stopLine())) > 2.5)
                {
                    if(proximity_deceleration_distance_required > 0 && light_deceleration_distance_required == 0)
                    {
                        startAcceleration(vehicle_, vehicle_->maxSpeed());
                    }
                }
            }
            else
            {
                if(abs(vehicle_->unitVector()[dot] * (vehicle_->exteriorPosition(FRONT_BUMPER)[dot] - vehicle_->stopLine())) < 2.5)
                {
                    if(light_deceleration_distance_required < 0.5 && proximity_deceleration_distance_required == 0)
                    {
                        startAcceleration(vehicle_, vehicle_->maxSpeed());
                    }
                }
            }
        }
        else
        {
            if(proximity_deceleration_distance_required == 0)
            {
                startAcceleration(vehicle_, vehicle_->maxSpeed());
            }
        }
    }
}

float Simulation::determineCloseProximityDecelerationDistance(Vehicle* vehicle_)
{
    float close_vehicle_distance = determineCloseVehicleDecelerationDistance(vehicle_);
    float lane_change_distance = 0;
    float brake_light_distance = determineBrakeLightDecelerationDistance(vehicle_);

    if(!(vehicle_->currentState() & IN_INTERSECTION) && 
       !(vehicle_->currentState() & THROUGH_INTERSECTION))
    {
        lane_change_distance = determineLaneChangeDecelerationDistance(vehicle_);
    }

    if(std::isnan(close_vehicle_distance) || std::isinf(close_vehicle_distance))
    {
        std::cout << vehicle_->number() << std::endl;
        std::cout << close_vehicle_distance << " " << __LINE__ << std::endl;
        throw;
    }
    if(std::isnan(lane_change_distance) || std::isinf(lane_change_distance))
    {
        std::cout << vehicle_->number() << std::endl;
        std::cout << lane_change_distance << " " << __LINE__ << std::endl;
        throw;
    }
    if(std::isnan(brake_light_distance) || std::isinf(brake_light_distance))
    {
        std::cout << vehicle_->number() << std::endl;
        std::cout << brake_light_distance << " " << __LINE__ << std::endl;
        throw;
    }

    //here we are looking to return the lowest non-zero stopping distance
    if(close_vehicle_distance != 0) 
    {
        //this means close vehicle distance is eligable
        if (lane_change_distance < close_vehicle_distance && lane_change_distance != 0)
        {
            //this means that lane change distance is eligable and close vehicle distance is no longer
            if(brake_light_distance < lane_change_distance && brake_light_distance != 0)
            {
                //this means that brake distance is the lowest non-zero stopping distance
                return brake_light_distance;
            }
            else
            {
                //this means that lane change is the lowest non-zero distance
                return lane_change_distance;
            }
        }
        else
        {
            //this means that lane change is not eligable
            if(brake_light_distance < close_vehicle_distance && brake_light_distance != 0)
            {
                //this means that brake light distance is the lowest non-zero distance
                return brake_light_distance;
            }
            else
            {
                //this means that neither lane change or brake light were eligible
                //which makes close distance the lowest non-zero distance by default
                return close_vehicle_distance;
            }
        }
    }
    else if(lane_change_distance != 0)
    {
        //this means that lane change distance is eligable but close vehicle distance is not
        if (brake_light_distance < lane_change_distance && brake_light_distance !=0)
        {
            //this means that brake light distance is the lowest non-zero distance
            return brake_light_distance;
        }
        else
        {
            //this means that lane change distance was the lowest non-zero value
            return lane_change_distance;
        }
    }
    else
    {
        //neither close vehicle distance or lane change distance were eligable
        //if brake light distance is 0 then we would be returning 0 anyways
        //and if brake light isn't zero it is automatically the smallest non-zero distance
        return brake_light_distance;
    }
}

float Simulation::determineCloseVehicleDecelerationDistance(Vehicle* vehicle_)
{
    float minimum_nonzero_distance = 0;
    for(uint32 i = 0; i < active_vehicles.size(); i++)
    {
        if(active_vehicles[i] == vehicle_)
        {
            continue;
        }
        float distance_ahead = distanceAhead(vehicle_, active_vehicles[i]);
        if (distance_ahead != 0)
        {
            if(minimum_nonzero_distance == 0 || distance_ahead < minimum_nonzero_distance)
            {
                minimum_nonzero_distance = distance_ahead;
            }
            if(std::isnan(minimum_nonzero_distance) || std::isinf(minimum_nonzero_distance))
            {
                std::cout << vehicle_->number() << " " << active_vehicles[i]->number() << std::endl;
                std::cout << minimum_nonzero_distance << " " << __LINE__ << std::endl;
                throw;
            }
        }
    }
    return minimum_nonzero_distance;
}

float Simulation::distanceAhead(Vehicle* current_vehicle_, Vehicle* test_vehicle_)
{
    bool dot = findComponent(current_vehicle_->exteriorPosition(FRONT_BUMPER), current_vehicle_->exteriorPosition(BACK_BUMPER));
    bool not_dot = !dot;

    //this may cause some issues, may need to be changed to external positions
    //int8 dot_modifier = current_vehicle_->currentVelocity()[dot] >= 0 ? 1 : -1;
    //int8 not_dot_modifier = current_vehicle_->currentVelocity()[not_dot] >= 0 ? 1 : -1;

    int8 dot_modifier = current_vehicle_->exteriorPosition(FRONT_BUMPER)[dot] >= current_vehicle_->exteriorPosition(BACK_BUMPER)[dot] ? 1 : -1;

    float slope = (current_vehicle_->exteriorPosition(FRONT_BUMPER)[not_dot] - current_vehicle_->exteriorPosition(BACK_BUMPER)[not_dot]) / 
                  (current_vehicle_->exteriorPosition(FRONT_BUMPER)[dot] - current_vehicle_->exteriorPosition(BACK_BUMPER)[dot]);
    
    float b_front_left = current_vehicle_->exteriorPosition(FRONT_LEFT)[not_dot] - (slope * current_vehicle_->exteriorPosition(FRONT_LEFT)[dot]);
    float b_front_right = current_vehicle_->exteriorPosition(FRONT_RIGHT)[not_dot] - (slope * current_vehicle_->exteriorPosition(FRONT_RIGHT)[dot]);

    bool greater = current_vehicle_->exteriorPosition(FRONT_LEFT)[not_dot] > current_vehicle_->exteriorPosition(FRONT_RIGHT)[not_dot];

    float lowest_nonzero_distance = 0;

    for(uint8 i = 0; i < TOTAL_POINTS; i++)
    {
        float test_value = test_vehicle_->exteriorPosition(i)[dot];
        float test_result = test_vehicle_->exteriorPosition(i)[not_dot];
        if(greater)
        {
            if((test_result > ((slope * test_value) + b_front_right)) &&
               (test_result < ((slope * test_value) + b_front_left)))
            {
                float distance_point[2];
                distance_point[not_dot] = test_result;
                if(slope != 0)
                {
                    distance_point[dot] = (test_result - b_front_right) / slope;
                }
                else
                {
                    distance_point[dot] = test_value;
                }
                float distance_between = distanceBetween(current_vehicle_->exteriorPosition(FRONT_RIGHT), distance_point);
                if(std::isnan(distance_between) || std::isinf(distance_between))
                {
                    std::cout << current_vehicle_->number() << " " << test_vehicle_->number() << std::endl;
                    std::cout << current_vehicle_->exteriorPosition(FRONT_RIGHT)[x] << " " << current_vehicle_->exteriorPosition(FRONT_RIGHT)[y] <<std::endl;
                    std::cout << distance_point[x] << " " << distance_point[y] << std::endl;
                    std::cout << distance_between << " " << __LINE__ << std::endl;
                    throw;
                }
                if(distance_between < current_vehicle_->slowingDistance() || distance_between != 0)
                {
                    if(lowest_nonzero_distance == 0 || distance_between < lowest_nonzero_distance)
                    {
                        lowest_nonzero_distance = distance_between;
                    }
                }
            }
        }
        else
        {
            if((test_result < ((slope * test_value) + b_front_right)) &&
               (test_result > ((slope * test_value) + b_front_left)))
            {
                float distance_point[2];
                distance_point[not_dot] = test_result;
                if(slope != 0)
                {
                    distance_point[dot] = (test_result - b_front_left) / slope;
                }
                else
                {
                    distance_point[dot] = test_value;
                }
                float distance_between = distanceBetween(current_vehicle_->exteriorPosition(FRONT_LEFT), distance_point);
                if(std::isnan(distance_between) || std::isinf(distance_between))
                {
                    std::cout << current_vehicle_->number() << " " << test_vehicle_->number() << std::endl;
                    std::cout << current_vehicle_->exteriorPosition(FRONT_LEFT)[x] << " " << current_vehicle_->exteriorPosition(FRONT_LEFT)[y] <<std::endl;
                    std::cout << distance_point[x] << " " << distance_point[y] << std::endl;
                    std::cout << distance_between << " " << __LINE__ << std::endl;
                    throw;
                }
                if(distance_between < current_vehicle_->slowingDistance() || distance_between != 0)
                {
                    if(lowest_nonzero_distance == 0 || distance_between < lowest_nonzero_distance)
                    {
                        lowest_nonzero_distance = distance_between;
                    }
                }
            }
        }
    }
    if(lowest_nonzero_distance == 0)
    {
        return 0;
    }
    if(lowest_nonzero_distance < current_vehicle_->mimumumStoppingDistance())
    {
        return lowest_nonzero_distance;
    }
    lowest_nonzero_distance -= current_vehicle_->mimumumStoppingDistance();
    if(lowest_nonzero_distance < 0)
    {
        //this will almost certainly result in a collision
        SWERRINT(current_vehicle_->number());
    }
    return lowest_nonzero_distance;
}

float Simulation::determineLaneChangeDecelerationDistance(Vehicle* vehicle_)
{
    if(vehicle_->currentState() & IN_INTERSECTION ||
       vehicle_->currentState() & THROUGH_INTERSECTION)
    {
        SWERRINT(vehicle_->currentState());
        return 0;
    }

    float changing_lane_distance = 0;
    float close_lane_change_distance = determineCloseLaneChangeDeceleration(vehicle_);

    if(!(vehicle_->currentState() & CORRECT_LANE))
    {
        changing_lane_distance = determineChangingLaneDeceleration(vehicle_);
    }

    if(changing_lane_distance < 0)
    {
        SWERRINT(vehicle_->number());
    }
    if(close_lane_change_distance < 0)
    {
        SWERRINT(my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->lanePath());
    }

    //if they're both the same it doesn't matter which is returned, if they're both zero then zero will be returned
    //otherwise we want the lower of the two
    return changing_lane_distance < close_lane_change_distance ? changing_lane_distance : close_lane_change_distance;
}

float Simulation::determineChangingLaneDeceleration(Vehicle* vehicle_)
{
    if(laneChangeDecelerationRequired(vehicle_))
    {
        float return_value = vehicle_->distanceToStopComfortably();
        if(return_value < 0)
        {
            return_value *= -1;
        }
        return return_value;
    }
    else
    {
        return 0;
    }
}

bool Simulation::laneChangeDecelerationRequired(Vehicle* vehicle_)
{
    if(vehicle_->currentState() & CORRECT_LANE)
    {
        return false;
    }
    //shoulderCheck returns true if it is clear to change lanes
    //therefore if it is true there is no need for acceleration and 
    //this should return false
    if (!shoulderCheck(vehicle_, changeLaneDirection(vehicle_)))
    {
        return true;
    }
    return false;
}

float Simulation::determineCloseLaneChangeDeceleration(Vehicle* vehicle_)
{
    path current_lane_path = my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->lanePath();
    switch(current_lane_path)
    {
        case(LEFT):
        {
            return (checkLaneBlinkerDistance(vehicle_, my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber() + 1), 1));
        }
            break;
        case(RIGHT):
        {
            return (checkLaneBlinkerDistance(vehicle_, my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber() - 1), -1));
        }
            break;
        case(STRAIGHT):
        {
            float right_blinker_distance = checkLaneBlinkerDistance(vehicle_, my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber() + 1), 1);
            float left_blinker_distance = checkLaneBlinkerDistance(vehicle_, my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber() - 1), -1);
            return  right_blinker_distance < left_blinker_distance ? right_blinker_distance : left_blinker_distance;
                   
        }
            break;
        default: SWERRINT(current_lane_path);
    }
    return 0;
}

float Simulation::checkLaneBlinkerDistance(Vehicle* vehicle_, Lane* lane_, int8 direction_)
{
     if(lane_->numberOfVehicles() < 1)
    {
        return false;
    }
    else
    {
        bool blinker_direction = direction_ > 0;
        bool dot = findComponent(vehicle_->exteriorPosition(FRONT_BUMPER), vehicle_->exteriorPosition(BACK_BUMPER));
        int8 modifier = vehicle_->exteriorPosition(FRONT_BUMPER)[dot] > vehicle_->exteriorPosition(BACK_BUMPER)[dot] ? 1 : -1;
        float separation = 0;
        for(uint32 i = 0; i < lane_->numberOfVehicles(); i++)
        {
            if(vehicle_list[lane_->vehicleAtIndex(i)]->blinker(blinker_direction))
            {
                if (modifier > 0)
                {
                    if(vehicle_->exteriorPosition(FRONT_BUMPER)[dot] < vehicle_list[lane_->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot] &&
                       vehicle_->exteriorPosition(FRONT_BUMPER)[dot] > (vehicle_list[lane_->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot] - vehicle_->slowingDistance()))
                    {
                        float distance = vehicle_list[lane_->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot] - vehicle_->exteriorPosition(FRONT_BUMPER)[dot];
                        if(separation == 0 || distance < separation)
                        {
                            separation = distance;
                        }
                    }
                }
                else
                {
                    if(vehicle_->exteriorPosition(FRONT_BUMPER)[dot] > vehicle_list[lane_->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot] &&
                       vehicle_->exteriorPosition(FRONT_BUMPER)[dot] < (vehicle_list[lane_->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot] + vehicle_->slowingDistance()))
                    {
                        float distance = vehicle_->exteriorPosition(FRONT_BUMPER)[dot] - vehicle_list[lane_->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot];
                        if(separation == 0 || distance < separation)
                        {
                            separation = distance;
                        }
                    }
                }
            }
        }
        return separation;
    }
}

float Simulation::determineBrakeLightDecelerationDistance(Vehicle* vehicle_)
{
    if(vehicle_->currentState() & IN_INTERSECTION)
    {
        return 0;
    }

    direction road_direction;

    if(vehicle_->currentState() & THROUGH_INTERSECTION)
    {
        road_direction = my_intersection.getRoad(vehicle_->vehicleDirection())->correspondingExit(vehicle_->vehiclePath());
    }
    else
    {
        road_direction = vehicle_->vehicleDirection();
    }

    Lane* current_lane = my_intersection.getRoad(road_direction)->getLane(vehicle_->laneNumber());

    bool dot = findComponent(vehicle_->exteriorPosition(FRONT_BUMPER), vehicle_->exteriorPosition(BACK_BUMPER));

    float lowest_nonzero_distance = 0;

    if(current_lane->numberOfVehicles() > 1)
    {
        for(uint32 i = 0; i < current_lane->numberOfVehicles(); i++)
        {
            if(current_lane->vehicleAtIndex(i) != vehicle_->number())
            {
                float distance_between = vehicle_->unitVector()[dot] * 
                                         (vehicle_list[current_lane->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot] - 
                                         vehicle_->exteriorPosition(FRONT_BUMPER)[dot]);
                if(distance_between > 0)
                {
                    if(lowest_nonzero_distance == 0 || lowest_nonzero_distance > distance_between)
                    {
                        lowest_nonzero_distance = distance_between;
                    }
                }
            }
        }
    }
    if(lowest_nonzero_distance == 0)
    {
        return 0;
    }
    if(lowest_nonzero_distance < vehicle_->mimumumStoppingDistance())
    {
        return lowest_nonzero_distance;
    }
    lowest_nonzero_distance -= vehicle_->mimumumStoppingDistance();
    if(lowest_nonzero_distance < 0)
    {
        //this will almost certainly result in a collision
        SWERRINT(vehicle_->number());
    }
    return lowest_nonzero_distance;
}


//light based deceleration code
float Simulation::determineLightBasedDecelerationDistance(Vehicle* vehicle_)
{
    if(vehicle_->currentState() & IN_INTERSECTION ||
       vehicle_->currentState() & THROUGH_INTERSECTION)
    {
        return 0;
    }

    float light_deceleration_distance;
    if(light_change_occured)
    {
        light_deceleration_distance = lightChangeDecelerationDistance(vehicle_);
    }
    else
    {
        light_deceleration_distance = lightColourDecelerationDistance(vehicle_);
    }

    //if a light change occurs then lightChangeDecelerationDistance takes priority since it can determine weather or not a vehicle
    //goes through on yellow (or red in emergency) wheras lightColourDecelerationDistance is used more when a vehicle is spawned into
    //a red or yellow light
    return light_deceleration_distance;
}

float Simulation::lightChangeDecelerationDistance(Vehicle* vehicle_)
{
    if(!light_change_occured)
    {
        SWERRINT(light_change_occured);
    }

    bool dot = findComponent(vehicle_->exteriorPosition(FRONT_BUMPER), vehicle_->exteriorPosition(BACK_BUMPER));

    int8 dot_modifier = vehicle_->exteriorPosition(FRONT_BUMPER)[dot] > vehicle_->exteriorPosition(BACK_BUMPER)[dot] ? 1 : -1;

    if(vehicle_->lightChange(my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection())))
    {
        float distance = dot_modifier * (vehicle_->stopLine() - vehicle_->currentPosition()[dot]);
        if(distance < 0)
        {
            SWERRINT(my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()));
        }
        return distance;
    }
    return 0;
}

float Simulation::lightColourDecelerationDistance(Vehicle* vehicle_)
{
    if(vehicle_->goingThroughLight())
    {
        return 0;
    }

    if(light_change_occured)
    {
        SWERRINT(light_change_occured);
        return 0;
    }

    bool dot = findComponent(vehicle_->exteriorPosition(FRONT_BUMPER), vehicle_->exteriorPosition(BACK_BUMPER));

    int8 dot_modifier = vehicle_->exteriorPosition(FRONT_BUMPER)[dot] > vehicle_->exteriorPosition(BACK_BUMPER)[dot] ? 1 : -1;

    if (my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()) == YELLOW ||
        my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()) == RED)
    {
        float distance = dot_modifier * (vehicle_->stopLine() - vehicle_->currentPosition()[dot]);
        if(std::isnan(distance) || std::isinf(distance))
        {
            std::cout << dot << vehicle_->number() << std::endl;
            std::cout << distance << " " << __LINE__ << std::endl;
            std::cout << vehicle_->stopLine() << " " << vehicle_->exteriorPosition(FRONT_BUMPER)[dot] << std::endl;
            throw;
        }
        if(distance < 0)
        {
            if(!vehicle_->goingThroughLight())
            {
                SWERRINT(vehicle_->currentState());
                SWERRINT(vehicle_->number());
                SWERRINT(dot_modifier);
            }
        }
        return distance;
    }
    return 0;
}


//changing acceleration/deceleration code
void Simulation::startAcceleration(Vehicle* vehicle_, float target_speed_)
{
    if(MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]) < target_speed_)
    {
        vehicle_->accelerate(target_speed_);
        if(vehicle_->currentAccelerationMagnitude() > 0)
        {
            if(!(vehicle_->currentState() & ACCELERATING))
            {
                changeState(vehicle_, ACCELERATING, ADD);
            }
            if (!(vehicle_->currentState() & DRIVING))
            {
                changeState(vehicle_, DRIVING, ADD);
                vehicle_->toggleBrakeLights(OFF);
            }
            if (vehicle_->currentState() & DECELERATING)
            {
                changeState(vehicle_, DECELERATING, REMOVE);
                vehicle_->toggleBrakeLights(OFF);
            }
        }
    }

}

void Simulation::startDeceleration(Vehicle* vehicle_, float target_speed_)
{
    if(MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]) > target_speed_)
    {
        if(!(vehicle_->currentState() & DECELERATING) &&
            (vehicle_->currentState() & DRIVING))
        {   
            vehicle_->accelerate(target_speed_);
            if (vehicle_->currentAccelerationMagnitude() > 0)
            {
                changeState(vehicle_, DECELERATING, ADD);
                if (vehicle_->currentState() & ACCELERATING)
                {
                    changeState(vehicle_, ACCELERATING, REMOVE);
                }
                vehicle_->toggleBrakeLights(ON);
            }
            else
            {
                SWERRFLOAT(vehicle_->currentAccelerationMagnitude());
            }
        }
    }
}

void Simulation::startDeceleration(Vehicle* vehicle_, float target_speed_, float distance_remaining_)
{
    if(MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]) > target_speed_)
    {
        if(vehicle_->currentState() & DRIVING)
        {   
            if(vehicle_->accelerate(target_speed_, distance_remaining_))
            {
                if (vehicle_->currentAccelerationMagnitude() > 0 && !(vehicle_->currentState() & DECELERATING))
                {
                    changeState(vehicle_, DECELERATING, ADD);
                    if (vehicle_->currentState() & ACCELERATING)
                    {
                        changeState(vehicle_, ACCELERATING, REMOVE);
                    }
                    vehicle_->toggleBrakeLights(ON);
                }
            }
            else
            {
                if(vehicle_->currentState() & DECELERATING)
                {
                    changeState(vehicle_, DECELERATING, REMOVE);
                    vehicle_->toggleBrakeLights(OFF);
                }
            }
        }
    }
}

void Simulation::changeDeceleration(Vehicle* vehicle_, float target_speed_)
{
    //this is used when we are already decelerating to decelerate to a new target speed
    if(MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]) > target_speed_)
    {
        if(vehicle_->currentState() & DECELERATING)
        {   
            vehicle_->accelerate(target_speed_);
            if (!(vehicle_->currentAccelerationMagnitude() > 0))
            {
                SWERRFLOAT(vehicle_->currentAccelerationMagnitude());
            }
        }
    }
}
void Simulation::changeDeceleration(Vehicle* vehicle_, float target_speed_, float distance_remaining_)
{
    //this is used when we are already decelerating to decelerate to a new target speed
    if(MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]) > target_speed_)
    {
        if(vehicle_->currentState() & DECELERATING)
        {   
            vehicle_->accelerate(target_speed_, distance_remaining_);
            if (!(vehicle_->currentAccelerationMagnitude() > 0))
            {
                SWERRFLOAT(vehicle_->currentAccelerationMagnitude());
            }
        }
    }
}

//Spawn related code
void Simulation::generateVehicle(uint32 number_)
{
    path vehicle_path;
    direction vehicle_direction;
    uint8 lane_number;
    DriverType driver_type;

    std::random_device root;
    std::mt19937 random_path(root());
    std::mt19937 random_direction(root());
    std::mt19937 random_lane_number(root());
    std::mt19937 random_driver_type(root());

    std::uniform_int_distribution<> path_distribution(1, 3);
    std::uniform_int_distribution<> direction_distribution(0, 3);
    std::uniform_int_distribution<> driver_type_distribution(1, 3);

    uint8 switch_placeholder = path_distribution(random_path);
    switch(switch_placeholder)
    {
        case(1): vehicle_path = LEFT;
            break;
        case(2): vehicle_path = STRAIGHT;
            break;
        case(3): vehicle_path = RIGHT;
            break;
        default: SWERRINT(switch_placeholder);
    }

    switch_placeholder = direction_distribution(random_direction);
    switch(switch_placeholder)
    {
        case(0):
        {
            vehicle_direction = NORTH;
            std::uniform_int_distribution<> lane_number_distribution(0, intersection_params.ns_number_of_entries - 1);
            lane_number = lane_number_distribution(random_lane_number) + intersection_params.ns_number_of_exits;
        }
            break;
        case(1):
        {
            vehicle_direction = SOUTH;
            std::uniform_int_distribution<> lane_number_distribution(0, intersection_params.ns_number_of_entries - 1);
            lane_number = lane_number_distribution(random_lane_number) + intersection_params.ns_number_of_exits;
        }
            break;
        case(2):
        {
            vehicle_direction = EAST;
            std::uniform_int_distribution<> lane_number_distribution(0, intersection_params.ew_number_of_entries - 1);
            lane_number = lane_number_distribution(random_lane_number) + intersection_params.ew_number_of_exits;
        }
            break;
        case(3):
        {
            vehicle_direction = WEST;
            std::uniform_int_distribution<> lane_number_distribution(0, intersection_params.ew_number_of_entries - 1);
            lane_number = lane_number_distribution(random_lane_number) + intersection_params.ew_number_of_exits;
        } 
            break;
        default: SWERRINT(switch_placeholder);
    }

    switch_placeholder = driver_type_distribution(random_driver_type);
    switch(switch_placeholder)
    {
        case(1): driver_type = CALM;
            break;
        case(2): driver_type = NORMAL;
            break;
        case(3): driver_type = AGGRESSIVE;
            break;
        default: SWERRINT(switch_placeholder);
    }
    vehicle_list[number_] = new Car(number_, vehicle_path, my_intersection.getRoad(vehicle_direction)->getLane(lane_number), driver_type);

    if(simulation_params.print_simulation_events)
    {
        printVehicleArrival(vehicle_list[number_]);
    }
    addToActiveVehicles(vehicle_list[number_]);
    my_intersection.getRoad(vehicle_list[number_]->vehicleDirection())->addToRoad(number_);
    my_intersection.getRoad(vehicle_list[number_]->vehicleDirection())->getLane(vehicle_list[number_]->laneNumber())->addToLane(number_);
}

bool Simulation::spawnVehicle()
{
    if(my_spawnTimer < 1)
    {
        return false;
    }
    else if(my_spawnTimer <= 5)
    {
        std::random_device root;
        std::mt19937 spawn_chance(root());
        double proability_percentage_per_second = simulation_params.spawn_density * pow((my_spawnTimer / 4), 2);
        double absolute_probability_per_time_step = (proability_percentage_per_second / 100) * (simulation_params.time_step);
        std::uniform_real_distribution<> probabiltity_distribution(0, 100);
        double random_value = probabiltity_distribution(spawn_chance) * 1000; //*1000 since it is a precent
        return random_value <= (absolute_probability_per_time_step * 100000); //*100000 since it is absolute
    }
    else
    {
        std::random_device root;
        std::mt19937 spawn_chance(root());
        std::uniform_real_distribution<> probabiltity_distribution(0, 100);
        double random_value = probabiltity_distribution(spawn_chance) * 1000; //*1000 since it is a percent
        return random_value <= simulation_params.spawn_density * 1000; //*1000 since it is also a percent
    }
}

bool Simulation::collisionAnalysis()
{
    for(uint32 i = 0; i < my_vehiclesMade - 1; i++)
    {
        for(uint32 j = i + 1; j < my_vehiclesMade; j++)
        {
            if(!(vehicle_list[i]->isCompleted()) && !(vehicle_list[j]->isCompleted()))
            {
                if(vehicle_list[i]->collisionCheck(vehicle_list[j]))
                {
                    printCollisionInformation(vehicle_list[i], vehicle_list[j]);
                    return true;
                }
            }
        }
    }
    return false;
}

//Vehicle List Code
Vehicle* Simulation::vehicleAtIndex(uint32 index_)
{
    try
    {
        if(index_ < active_vehicles.size() && index_ >= 0)
        {
            return active_vehicles[index_];
        }
        else
        {
            throw(std::out_of_range("Out of Vehicle List Bounds"));
        }
    }
    catch (const std::out_of_range &Out_of_Range)
    {
        //hard SWERR
        SWERRINT(index_);
        throw;
    }
}

uint32 Simulation::indexOfVehicle(Vehicle* vehicle_)
{
    if(active_vehicles.size() > 0)
    {
        for(uint32 i = 0; i < active_vehicles.size(); i++)
        {
            if(active_vehicles[i] == vehicle_)
            {
                return i;
            }
        }
    }
    SWERRINT(-1);
    return 0;
}

bool Simulation::isActive(Vehicle* vehicle_)
{
    if(active_vehicles.size() > 0)
    {
        for(uint32 i = 0; i < active_vehicles.size(); i++)
        {
            if(active_vehicles[i] == vehicle_)
            {
                return true;
            }
        }
    }
    return false;
}

void Simulation::addToActiveVehicles(Vehicle* vehicle_)
{
    active_vehicles.insert(active_vehicles.begin(), vehicle_);
}

bool Simulation::removeFromActiveVehicles(Vehicle* vehicle_)
{
    if(isActive(vehicle_))
    {
        active_vehicles.erase(active_vehicles.begin() + indexOfVehicle(vehicle_));
        return true;
    }
    else
    {
        SWERRINT(vehicle_->number()<<8 + vehicle_->currentState());
        return false;
    }
}

void Simulation::calculateAverages()
{
    for (uint8 i = 0; i < TOTAL_AVERAGES; i++)
    {
        for(uint32 j = 0; j < simulation_params.number_of_vehicles; j++)
        {
            switch(i)
            {
                case(TIME_THROUGH_INTERSECTION): averages[i] += vehicle_list[j]->totalTime();
                    break;
                case(TIME_IN_INTERSECTION): averages[i] += vehicle_list[j]->timeInIntersection();
                    break;
                case(TIME_AT_MAX_SPEED): averages[i] += vehicle_list[j]->timeAtMaxSpeed();
                    break;
                case(TIME_STOPPED): averages[i] += vehicle_list[j]->timeStopped();
                    break;
                case(TIME_BETWEEN_SPAWNS): continue; //already calculated
                    break;
                case(FUEL_CONSUMPTION): averages[i] += vehicle_list[j]->fuelConsumed();
                    break;
                default: SWERRINT(i);
            }
        }
        if(i == TIME_BETWEEN_SPAWNS)
        {
            averages[i] /= my_vehiclesMade;
        }
        else
        {  
            averages[i] /= simulation_params.number_of_vehicles;
        }
    }
}

severity Simulation::calculateCollisionSeverity(Vehicle* first_vehicle_, Vehicle* second_vehicle_)
{
    float x_velocity_difference = second_vehicle_->currentVelocity()[x] - first_vehicle_->currentVelocity()[x];
    float y_velocity_difference = second_vehicle_->currentVelocity()[y] - first_vehicle_->currentVelocity()[y];

    float velocity_difference = MAGNITUDE(x_velocity_difference, y_velocity_difference);

    if (velocity_difference >= 60)
    {
        //this is a head on collision on a 400 series highway
        //corresponds to probable death
        return FATAL;
    }
    else if( velocity_difference >= 50)
    {
        //this is a head on collision on a non-400 series highway
        //corresponds to serious injury or death
        return HIGH;
    }
    else if(velocity_difference >= 40)
    {
        //this is a head on collision on a high speed limit regular road
        //corresponds to moderate or severe injury
        return MODERATE;
    }
    else if (velocity_difference >= 30)
    {
        //this is a head on collision on a standard road
        //corresponds to minor or moderate injury
        return MEDIUM;
    }
    else if(velocity_difference >= 15)
    {
        //this is a head on collision at low speed
        //corresponds to no injury or minor injury
        return MINOR;
    }
    else
    {
        //this is a fender bender
        //corresponds to no injury
        return LOW;
    }
}

direction Simulation::opposingDirection(direction direction_)
{
    switch(direction_)
    {
        case(NORTH): return SOUTH;
            break;
        case(SOUTH): return NORTH;
            break;
        case(EAST): return WEST;
            break;
        case(WEST): return EAST;
            break;
        default: SWERRINT(direction_);
    }
    return NORTH;
}

//Printing funtions, no need for explanation
//They print things
void Simulation::printCompletion(Vehicle* vehicle_)
{
    events << elapsed_time << " Vehicle " << vehicle_->number() << " has completed the intersection" << std::endl;
}

void Simulation::printResults()
{
    

    //This will become much more elaborate once
    //multiple vehicles has been added in
    //should eventually display average times for various
    //actions
    calculateAverages();
    results.open("./Output/Results.txt");
    results << "~~~~~~~~" << std::endl;
    results << "RESULTS" << std::endl;
    results << "~~~~~~~~" << std::endl;

    results << std::endl << std::endl;

    for(uint8 i = 0; i < TOTAL_AVERAGES; i++)
    {
        results << AVERAGE_STR[i] << " " << AVERAGE_UNITS_STR[i] << ":\t" << averages[i] << std::endl; 
    }
    results << std::endl << std::endl;
    results << "************************" << std::endl;
    results << "Fuel Economy: " << GRADE_STR[fuelConsumptionGrade(averages[FUEL_CONSUMPTION])] << std::endl;
    results << "************************" << std::endl;
    results.close();
}

void Simulation::printLaneChange(Vehicle* vehicle_, uint8 new_lane_)
{
    events << elapsed_time << " Vehicle " << vehicle_->number() << ": changed from lane " << (int)vehicle_->laneNumber() << " to " << (int)new_lane_ << std::endl;
}

void Simulation::printTrafficLightStateChange(TrafficLight* traffic_light_)
{
    events << elapsed_time << " Traffic light now in state: " << LIGHT_EVENT_STATE_STR[traffic_light_->currentEvent()] << " (" << traffic_light_->state() << ")" << std::endl;
}

void Simulation::printVehicleArrival(Vehicle* vehicle_)
{
    events << elapsed_time << " Vehicle " << vehicle_->name() << " has arrived from " << DIRECTION_STR[vehicle_->vehicleDirection()] << " heading " << PATH_STR[vehicle_->vehiclePath()] << std::endl;
}

void Simulation::printCollisionInformation(Vehicle* first_vehicle_, Vehicle* second_vehicle_)
{
    std::ofstream collision;
    std::string file_name = "./Output/CollisionInformation.txt";
    collision.open(file_name);

    collision << first_vehicle_->number() << "\t" << DRIVER_TYPE_STR[first_vehicle_->driverType()] << "\t";
    collision << DIRECTION_STR[first_vehicle_->vehicleDirection()] << "\t" << PATH_STR[first_vehicle_->vehiclePath()] << "\t" << VEHICLE_TYPE_STR[first_vehicle_->vehicleType()] << std::endl;
    collision << (int)first_vehicle_->currentState() << "\t" << first_vehicle_->currentPosition()[x] << "\t" << first_vehicle_->currentPosition()[y] << "\t";
    collision << first_vehicle_->currentVelocity()[x] << "\t" << first_vehicle_->currentVelocity()[y] << "\t";
    collision << first_vehicle_->currentAccelerationMagnitude() << "\t" << first_vehicle_->currentAcceleration()[x] << "\t" << first_vehicle_->currentAcceleration()[y] << "\t";
    for (uint8 k = 0; k < TOTAL_POINTS; k++)
    {
        collision << first_vehicle_->exteriorPosition(k)[x] << "\t" << first_vehicle_->exteriorPosition(k)[y] << "\t";
    }
    collision << std::endl;

    collision << second_vehicle_->number() << "\t" << DRIVER_TYPE_STR[second_vehicle_->driverType()] << "\t";
    collision << DIRECTION_STR[second_vehicle_->vehicleDirection()] << "\t" << PATH_STR[second_vehicle_->vehiclePath()] << "\t" << VEHICLE_TYPE_STR[second_vehicle_->vehicleType()] << std::endl;
    collision << (int)second_vehicle_->currentState() << "\t" << second_vehicle_->currentPosition()[x] << "\t" << second_vehicle_->currentPosition()[y] << "\t";
    collision << second_vehicle_->currentVelocity()[x] << "\t" << second_vehicle_->currentVelocity()[y] << "\t";
    collision << second_vehicle_->currentAccelerationMagnitude() << "\t" << second_vehicle_->currentAcceleration()[x] << "\t" << second_vehicle_->currentAcceleration()[y] << "\t";
    for (uint8 k = 0; k < TOTAL_POINTS; k++)
    {
        collision << second_vehicle_->exteriorPosition(k)[x] << "\t" << second_vehicle_->exteriorPosition(k)[y] << "\t";
    }
    collision << std::endl << std::endl << std::endl;

    collision << "Current Light State: " << LIGHT_EVENT_STATE_STR[my_intersection.trafficLight()->currentEvent()] << " (" << my_intersection.trafficLight()->state() <<")" << std::endl;
    collision << "Ran Light: " << first_vehicle_->goingThroughLight() << "\t" << second_vehicle_->goingThroughLight() << std::endl << std::endl;

    collision << "Number of Active Vehicles: " << active_vehicles.size() << std::endl;
    collision << "Number of Vehicles Made: " << my_vehiclesMade << std::endl << std::endl;

    collision << "Elapesed Time: " << elapsed_time << std::endl;

    collision << "Vehicle " << first_vehicle_->number() << " Times: " << first_vehicle_->totalTime() << "\t" << first_vehicle_->timeInIntersection() << "\t";
    collision << first_vehicle_->timeAtMaxSpeed() << "\t" << first_vehicle_->timeStopped() << "\t" << std::endl;

    collision << "Vehicle " << second_vehicle_->number() << " Times: " << second_vehicle_->totalTime() << "\t" << second_vehicle_->timeInIntersection() << "\t";
    collision << second_vehicle_->timeAtMaxSpeed() << "\t" << second_vehicle_->timeStopped() << "\t" << std::endl << std::endl;

    collision << "Brakelights Active: " << first_vehicle_->brakeLights() << "\t" << second_vehicle_->brakeLights() << std::endl;
    collision << "Blinkers Active: " << first_vehicle_->blinker(0) << " " << first_vehicle_->blinker(1) << "\t";
    collision << second_vehicle_->blinker(0) << " " << second_vehicle_->blinker(1) << std::endl;

    collision << std::endl << std::endl << std::endl;

    collision << "*******************************************************" << std::endl;
    collision << "Severity: " << SEVERITY_STR[calculateCollisionSeverity(first_vehicle_, second_vehicle_)] << std::endl;
    collision << "*******************************************************" << std::endl;

    collision.close();
}