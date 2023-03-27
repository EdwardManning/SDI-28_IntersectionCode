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
    debug_log.open("./Output/DebugLog.txt");
    my_vehiclesMade = 0;
    my_selfDrivingVehiclesMade = 0;
    my_leftVehiclesMade = 0;
    my_sdvLeftVehiclesMade = 0;
    my_straightVehiclesMade = 0;
    my_sdvStraightVehiclesMade = 0;
    my_rightVehiclesMade = 0;
    my_sdvRightVehiclesMade = 0;
    for(uint8 i = 0; i < TOTAL_AVERAGES; i++)
    {
        averages[i] = 0;
        left_averages[i] =0;
        straight_averages[i] = 0;
        right_averages[i] = 0;
        self_driving_averages[i] = 0;
        sdv_left_averages[i] = 0;
        sdv_straight_averages[i] = 0;
        sdv_right_averages[i] = 0;
        human_driving_averages[i] = 0;
        hd_left_averages[i] = 0;
        hd_straight_averages[i] = 0;
        hd_right_averages[i] = 0;
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
    debug_log.close();
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
            if (active_vehicles[i]->vehicleType() == CAR || 
                active_vehicles[i]->vehicleType() == SELF_DRIVING_CAR)
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
        if(simulation_params.print_results_for_python)
        {
            printResultsForPython();
        }
        std::cout << "Completed" << std::endl;
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
    bool start_turn = false;
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
            if(simulation_params.print_debug_acceleration)
            {
                debug_log << elapsed_time << " Vehicle " << vehicle_->number() << " has begun to turn with a velocity of ";
                debug_log << vehicle_->currentVelocity()[x] << " " << vehicle_->currentVelocity()[y];
                debug_log << " and state: " << (int)vehicle_->currentState() << std::endl;  
                start_turn = true;
            }
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
            if(simulation_params.print_debug_acceleration)
            {
                debug_log << elapsed_time << " Vehicle " << vehicle_->number();
                debug_log << " has exited the intersection with a velocity of: ";
                debug_log << vehicle_->currentVelocity()[x] << " " << vehicle_->currentVelocity()[y];
                debug_log << " and state: " << (int)vehicle_->currentState() << std::endl;  
            }
        }
        else if (vehicle_->currentState() & TURNING)
        {
            vehicle_->turn();
            if(simulation_params.print_debug_acceleration && start_turn)
            {
                debug_log << elapsed_time << " Vehicle " << vehicle_->number() << " is turning with a velocity of ";
                debug_log << vehicle_->currentVelocity()[x] << " " << vehicle_->currentVelocity()[y];
                debug_log << " and state: " << (int)vehicle_->currentState() << std::endl;  
            } 
        }
    }

    accelerate(vehicle_);
    if(simulation_params.print_debug_acceleration && start_turn)
    {
        debug_log << elapsed_time << " Vehicle " << vehicle_->number() << " is turning with a velocity of ";
        debug_log << vehicle_->currentVelocity()[x] << " " << vehicle_->currentVelocity()[y];
        debug_log << " post acceleration, state: " << (int)vehicle_->currentState() << std::endl; 
    }
    
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
    if(simulation_params.print_debug_acceleration && start_turn)
    {
        debug_log << elapsed_time << " Vehicle " << vehicle_->number() << " is going into second acceleration with state: ";
        debug_log << (int)vehicle_->currentState() << " and velocity: "; 
        debug_log << vehicle_->currentVelocity()[x] << " " << vehicle_->currentVelocity()[y] << std::endl;
    }
    if ((vehicle_->currentState() & ACCELERATING) | (vehicle_->currentState() & DECELERATING))
    {
        if(vehicle_->accelerate())
        {
            if (vehicle_->currentState() & DECELERATING)
            {
                changeState(vehicle_, DECELERATING, REMOVE);
                if(MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]) == 0)
                {
                    changeState(vehicle_, DRIVING, REMOVE);
                }
                else
                { 
                    vehicle_->toggleBrakeLights(OFF);
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
    if(simulation_params.print_debug_acceleration && start_turn)
    {
        debug_log << elapsed_time << " Vehicle " << vehicle_->number() << " is turning with a velocity of ";
        debug_log << vehicle_->currentVelocity()[x] << " " << vehicle_->currentVelocity()[y];
        debug_log << " post second acceleration, state: " << (int)vehicle_->currentState() << std::endl;
        start_turn = false; 
    }
    if((MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]) < 0.005) &&
       (vehicle_->currentState() & DECELERATING))
    {
        changeState(vehicle_, DRIVING, REMOVE);
        changeState(vehicle_, DECELERATING, REMOVE);
    }
    try
    {
        vehicle_->drive();
    }
    catch(vehicle_time_fail& vtf)
    {
        //hard swerr
        SWERRSTR(vtf.what());
        printVehicleFailInformation(vehicle_);
        throw vtf;
    }
    catch(impossible_state_fail& ist)
    {
        //hard swerr
        SWERRSTR(ist.what());
        printVehicleFailInformation(vehicle_);
        throw ist;
    }
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
    if(simulation_params.print_debug_acceleration && 
      ((proximity_deceleration_distance_required != 0) ||
      (light_deceleration_distance_required != 0)))
    {
        debug_log << elapsed_time << " Vehicle " << vehicle_->number() << " proximityDeceleration/lightAcceleration ";
        debug_log << proximity_deceleration_distance_required << "/" << light_deceleration_distance_required << std::endl;
    }
    if(vehicle_->currentState() & IN_INTERSECTION &&
       (my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()) != GREEN &&
       my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()) != YELLOW))
    {
        startAcceleration(vehicle_, vehicle_->maxSpeed());
        return;
    }

    if(vehicle_->vehiclePath() == LEFT)
    {
        bool dot = findComponent(vehicle_->exteriorPosition(FRONT_BUMPER), vehicle_->exteriorPosition(BACK_BUMPER));
        int8 modifier = vehicle_->exteriorPosition(FRONT_BUMPER)[dot] < vehicle_->exteriorPosition(BACK_BUMPER)[dot] ? -1 : 1;
        float stop_position = vehicle_->stopLine() + (modifier * (vehicle_->turnRadius()[dot] - vehicle_->turnRadius()[!dot]));
        float distance = modifier * (stop_position - vehicle_->exteriorPosition(FRONT_BUMPER)[dot]);
        if (vehicle_->currentState() & TURNING)
        {
            if(!checkTurnClear(vehicle_))
            {
                if((modifier < 0 && (vehicle_->exteriorPosition(FRONT_BUMPER)[dot] < stop_position)) ||
                   (modifier > 0 && vehicle_->exteriorPosition(FRONT_BUMPER)[dot] > stop_position))
                {
                    startAcceleration(vehicle_, vehicle_->maxSpeed());
                }
                else
                {
                    startDeceleration(vehicle_, STOP, distance);
                }
                return;
            }
        }
        else
        {
            if(!(vehicle_->currentState() & THROUGH_INTERSECTION))
            {
                if(!checkTurnClear(vehicle_) &&
                   my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()) == GREEN)
                {
                    if(proximity_deceleration_distance_required == 0 &&
                       light_deceleration_distance_required == 0)
                    {
                        // bool dot = findComponent(vehicle_->exteriorPosition(FRONT_BUMPER), vehicle_->exteriorPosition(BACK_BUMPER));
                        // float distance = vehicle_->stopLine() - vehicle_->exteriorPosition(FRONT_BUMPER)[dot];
                        // if(vehicle_->exteriorPosition(FRONT_BUMPER)[dot] < vehicle_->exteriorPosition(BACK_BUMPER)[dot])
                        // {
                        //     distance *= -1;
                        // }
                        // distance += intersection_params.lane_width;
                        startDeceleration(vehicle_, STOP, distance);
                        return;
                    }
                }
            }
        }
    }

    //std::cout << proximity_deceleration_distance_required << " " << light_deceleration_distance_required << std::endl;

    if(vehicle_->currentState() & DRIVING)
    {
        if(proximity_deceleration_distance_required != 0)
        {
            if(proximity_deceleration_distance_required > light_deceleration_distance_required && light_deceleration_distance_required != 0)
            {
                //if light deceleration distance is lower than the proximity deceleration distance and non zero
                if(!startDeceleration(vehicle_, STOP, light_deceleration_distance_required))
                {
                    if(simulation_params.print_debug_info)
                    {
                        debug_log << elapsed_time << " Deceleration Rejected " << vehicle_->number() << "\t" << light_deceleration_distance_required << "\t" << __LINE__ << std::endl;
                    }
                    if(!startDeceleration(vehicle_, STOP, proximity_deceleration_distance_required))
                    {
                        debug_log << elapsed_time << " Deceleration Rejected " << vehicle_->number() << "\t" << proximity_deceleration_distance_required << "\t" << __LINE__ << std::endl;
                        startAcceleration(vehicle_, vehicle_->maxSpeed());
                    }
                    else
                    {
                        if(simulation_params.print_debug_acceleration)
                        {
                            debug_log << elapsed_time << " Vehicle " << vehicle_->number();
                            debug_log << " is accelerating due to proximity: " << proximity_deceleration_distance_required;
                            debug_log << "\t" << __LINE__  << std::endl;
                        }
                    }
                }
                else
                {
                    if(simulation_params.print_debug_acceleration)
                    {
                        debug_log << elapsed_time << " Vehicle " << vehicle_->number();
                        debug_log << " is accelerating due to light: " << light_deceleration_distance_required;
                        debug_log << "\t" << __LINE__  << std::endl;
                    }
                }
            }
            else
            {
                //proximity deceleration distance is not zero and light deceleration distance is either zero or greater than it
                if(!startDeceleration(vehicle_, STOP, proximity_deceleration_distance_required))
                {
                    if(simulation_params.print_debug_info)
                    {
                        debug_log << elapsed_time << " Deceleration Rejected " << vehicle_->number() << "\t" << proximity_deceleration_distance_required << "\t" << __LINE__ << std::endl;
                    }
                    if(light_deceleration_distance_required != 0)
                    {
                        if(!startDeceleration(vehicle_, STOP, light_deceleration_distance_required))
                        {
                            debug_log << elapsed_time << " Deceleration Rejected " << vehicle_->number() << "\t" << light_deceleration_distance_required << "\t" << __LINE__ << std::endl;
                            startAcceleration(vehicle_, vehicle_->maxSpeed());
                        }
                        else
                        {
                            if(simulation_params.print_debug_acceleration)
                            {
                                debug_log << elapsed_time << " Vehicle " << vehicle_->number();
                                debug_log << " is accelerating due to light: " << light_deceleration_distance_required;
                                debug_log << "\t" << __LINE__  << std::endl;
                            }
                        }
                    }
                    else
                    {
                        startAcceleration(vehicle_, vehicle_->maxSpeed());
                    }
                }
                else
                {
                    if(simulation_params.print_debug_acceleration)
                    {
                        debug_log << elapsed_time << " Vehicle " << vehicle_->number();
                        debug_log << " is accelerating due to proximity: " << proximity_deceleration_distance_required;
                        debug_log << "\t" << __LINE__  << std::endl;
                    }
                }
            }
        }
        else if (light_deceleration_distance_required != 0)
        {
            //if proximity deceleration distance is zero and light deceleration is non zero then light deceleration is  
            //automatically the least non-zero deceleration distance
            if(!startDeceleration(vehicle_, STOP, light_deceleration_distance_required))
            {
                if(simulation_params.print_debug_info)
                {
                    debug_log << elapsed_time << " Deceleration Rejected " << vehicle_->number() << "\t" << light_deceleration_distance_required << "\t" << __LINE__ << std::endl;
                }
                startAcceleration(vehicle_, vehicle_->maxSpeed());
            }
            else
            {
                if(simulation_params.print_debug_acceleration)
                {
                    debug_log << elapsed_time << " Vehicle " << vehicle_->number();
                    debug_log << " is accelerating due to light: " << light_deceleration_distance_required;
                    debug_log << "\t" << __LINE__  << std::endl;
                }
            }
        }
        else //both are zero
        {
            //we can speed up
            startAcceleration(vehicle_, vehicle_->maxSpeed());
        }
    }
    else //stopped
    {
        if(!(vehicle_->currentState() & IN_INTERSECTION) && !(vehicle_->currentState() & THROUGH_INTERSECTION))
        {
            bool dot = findComponent(vehicle_->exteriorPosition(FRONT_BUMPER), vehicle_->exteriorPosition(BACK_BUMPER));
            int8 dot_modifier = vehicle_->exteriorPosition(FRONT_BUMPER)[dot] > vehicle_->exteriorPosition(BACK_BUMPER)[dot] ? 1 : -1;

            if(my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()) == GREEN)
            {
                float distance_left = dot_modifier * (vehicle_->stopLine() - vehicle_->currentPosition()[dot]);
                if(distance_left < vehicle_params.vehicle_length)
                {
                    //no car in front and green light
                    startAcceleration(vehicle_, vehicle_->maxSpeed());
                }
                else
                {
                    if(proximity_deceleration_distance_required > 0.25)
                    {
                        //car in front is accelerating into intersection
                        startAcceleration(vehicle_, vehicle_->maxSpeed());
                    }
                }
            }
            // else
            // {

            // }
            // if(!light_change_occured)
            // {
            //     if(abs(vehicle_->unitVector()[dot] * (vehicle_->exteriorPosition(FRONT_BUMPER)[dot] - vehicle_->stopLine())) > 2.5)
            //     {
            //         if(proximity_deceleration_distance_required > 0 && light_deceleration_distance_required == 0)
            //         {
            //             startAcceleration(vehicle_, vehicle_->maxSpeed());
            //         }
            //     }
            // }
            // else
            // {
            //     if(abs(vehicle_->unitVector()[dot] * (vehicle_->exteriorPosition(FRONT_BUMPER)[dot] - vehicle_->stopLine())) < 2.5)
            //     {
            //         if(light_deceleration_distance_required < 0.5 && proximity_deceleration_distance_required == 0)
            //         {
            //             startAcceleration(vehicle_, vehicle_->maxSpeed());
            //         }
            //     }
            // }
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

bool Simulation::checkTurnClear(Vehicle* vehicle_)
{
    for(uint32 i = 0; i < my_intersection.numberOfVehicles(); i++)
    {
        uint32 checked_vehicle = my_intersection.vehicleAtIndex(i);
        if(vehicle_list[checked_vehicle] != vehicle_)
        {
            if(vehicle_list[checked_vehicle]->vehicleDirection() != vehicle_->vehicleDirection())
            {
                if(vehicle_list[checked_vehicle]->vehiclePath() == STRAIGHT)
                {
                    return false;
                }
            }
        }
    }
    if(my_intersection.trafficLight()->currentLightColour(opposingDirection(vehicle_->vehicleDirection())) == GREEN)
    {
        for(uint32 i = 0; i < my_intersection.getRoad(opposingDirection(vehicle_->vehicleDirection()))->numberOfVehicles(); i++)
        {
            uint32 checked_vehicle = my_intersection.getRoad(opposingDirection(vehicle_->vehicleDirection()))->vehicleAtIndex(i);
            if(vehicle_list[checked_vehicle] != vehicle_)
            {
                if(vehicle_list[checked_vehicle]->vehiclePath() == STRAIGHT)
                {
                    bool dot = findComponent(vehicle_list[checked_vehicle]->exteriorPosition(FRONT_BUMPER), vehicle_list[checked_vehicle]->exteriorPosition(BACK_BUMPER));
                    float distance_remaining = vehicle_list[checked_vehicle]->stopLine() - vehicle_list[checked_vehicle]->currentPosition()[dot];
                    if(distance_remaining < 0)
                    {
                        distance_remaining *= -1;
                    }
                    if(distance_remaining < (intersection_params.lane_length / 2))
                    {
                        return false;
                    }
                }
            }
        }
    }
    return true;
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

    try
    {
        if(std::isnan(close_vehicle_distance) || std::isinf(close_vehicle_distance))
        {
            //hard swerr
            SWERRFLOAT(close_vehicle_distance);
            SWERRFLOAT(lane_change_distance);
            SWERRFLOAT(brake_light_distance);
            throw impossible_value_fail();
        }
        if(std::isnan(lane_change_distance) || std::isinf(lane_change_distance))
        {
            //hard swerr
            SWERRFLOAT(close_vehicle_distance);
            SWERRFLOAT(lane_change_distance);
            SWERRFLOAT(brake_light_distance);
            throw impossible_value_fail();
        }
        if(std::isnan(brake_light_distance) || std::isinf(brake_light_distance))
        {
            //hard swerr
            SWERRFLOAT(close_vehicle_distance);
            SWERRFLOAT(lane_change_distance);
            SWERRFLOAT(brake_light_distance);
            throw impossible_value_fail();
        }
    }
    catch(impossible_value_fail& ivf)
    {
        //hard swerr
        SWERRSTR(ivf.what());
        printVehicleFailInformation(vehicle_);
        throw ivf;
    }

    if(close_vehicle_distance > intersection_params.frame_length)
    {
        SWERRFLOAT(close_vehicle_distance);
    }
    if(lane_change_distance > intersection_params.frame_length)
    {
        SWERRFLOAT(lane_change_distance);
    }
    if(brake_light_distance > intersection_params.frame_length)
    {
        SWERRFLOAT(brake_light_distance);
    }
    
    if(simulation_params.print_debug_acceleration &&
      ((close_vehicle_distance != 0) ||
      (lane_change_distance != 0) ||
      (brake_light_distance != 0)))
    {
        debug_log << elapsed_time << " Vehicle " << vehicle_->number();
        debug_log << " close_vehicle_distance/lane_change_distance/brake_light_distance: ";
        debug_log << close_vehicle_distance << "/" << lane_change_distance << "/" << brake_light_distance;
        debug_log << std::endl;
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
            try
            {
                if(std::isnan(minimum_nonzero_distance) || std::isinf(minimum_nonzero_distance))
                {
                    //hard swerr
                    SWERRINT(active_vehicles[i]->number());
                    SWERRFLOAT(minimum_nonzero_distance);
                    throw impossible_value_fail();
                }
            }
            catch(impossible_value_fail& ivf)
            {
                //hard swerr
                SWERRSTR(ivf.what());
                printVehicleFailInformation(vehicle_);
                throw ivf;
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
        if(dot_modifier * (test_value - current_vehicle_->exteriorPosition(FRONT_RIGHT)[dot]) < 0)
        {
            //this was added so if there is a vehicle behind you it does not start the deceleration
            //if you are headed in the negative direction then the 
            continue;
        }
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
                try
                {
                    if(std::isnan(distance_between) || std::isinf(distance_between))
                    {
                        //hard swerr
                        SWERRINT(test_vehicle_->number());
                        SWERRFLOAT(distance_between);
                        throw impossible_value_fail();
                    }
                    
                }
                catch(impossible_value_fail& ivf)
                {
                    //hard swerr
                    SWERRSTR(ivf.what());
                    printVehicleFailInformation(current_vehicle_);
                    throw ivf;
                }
                if(distance_between < current_vehicle_->slowingDistance())
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
                try
                {
                    if(std::isnan(distance_between) || std::isinf(distance_between))
                    {
                        //hard swerr
                        SWERRINT(test_vehicle_->number());
                        SWERRFLOAT(distance_between);
                        throw impossible_value_fail();
                    }
                    
                }
                catch(impossible_value_fail& ivf)
                {
                    //hard swerr
                    SWERRSTR(ivf.what());
                    printVehicleFailInformation(current_vehicle_);
                    throw ivf;
                }
                if(distance_between < current_vehicle_->slowingDistance())
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
    if(light_change_occured && 
      (my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()) != 
       my_intersection.trafficLight()->previousLightColour(vehicle_->vehicleDirection())))
    {
        light_deceleration_distance = lightChangeDecelerationDistance(vehicle_);
        if(simulation_params.print_debug_acceleration && (light_deceleration_distance != 0))
        {
            debug_log << elapsed_time << " Vehicle " << vehicle_->number();
            debug_log << " light change deceleration distance: ";
            debug_log << light_deceleration_distance << std::endl;
        }
    }
    else
    {
        light_deceleration_distance = lightColourDecelerationDistance(vehicle_);
        if(simulation_params.print_debug_acceleration && (light_deceleration_distance != 0))
        {
            debug_log << elapsed_time << " Vehicle " << vehicle_->number();
            debug_log << " light colour deceleration distance: ";
            debug_log << light_deceleration_distance << std::endl;
        }
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
    if(vehicle_->goingThroughLight() && simulation_params.print_debug_info)
    {
        debug_log << elapsed_time << " Vehicle " << vehicle_->number() << " is going through the light\t" << vehicle_->currentVelocity()[x] << "\t" << vehicle_->currentVelocity()[y] << "\t";
        debug_log << dot_modifier * (vehicle_->stopLine() - vehicle_->currentPosition()[dot]) << std::endl;
    }
    return 0;
}

float Simulation::lightColourDecelerationDistance(Vehicle* vehicle_)
{
    if(vehicle_->goingThroughLight())
    {
        return 0;
    }

    if(light_change_occured && 
      (my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()) != 
       my_intersection.trafficLight()->previousLightColour(vehicle_->vehicleDirection())))
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
        try
        {
            if(std::isnan(distance) || std::isinf(distance))
            {
                //hard swerr
                SWERRINT(vehicle_->stopLine());
                SWERRFLOAT(distance);
                throw impossible_value_fail();
            }
            
        }
        catch(impossible_value_fail& ivf)
        {
            //hard swerr
            SWERRSTR(ivf.what());
            printVehicleFailInformation(vehicle_);
            throw ivf;
        }
        if(distance < 0)
        {
            if(!vehicle_->goingThroughLight())
            {
                SWERRINT(vehicle_->currentState());
                SWERRINT(vehicle_->number());
                SWERRINT(dot_modifier);
            }
            else
            {
                return 0;
            }
        }
        if(distance > intersection_params.lane_length)
        {
            SWERRFLOAT(distance);
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
        if((vehicle_->currentState() & DRIVING))
        {   
            vehicle_->accelerate(target_speed_);
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
    }
}

bool Simulation::startDeceleration(Vehicle* vehicle_, float target_speed_, float distance_remaining_)
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
                return true;
            }
            else
            {
                if(vehicle_->currentState() & DECELERATING)
                {
                    changeState(vehicle_, DECELERATING, REMOVE);
                    vehicle_->toggleBrakeLights(OFF);
                }
                return false;
            }
        }
    }
    return false;
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
    VehicleType vehicle_type;

    std::random_device root;
    std::mt19937 random_path(root());
    std::mt19937 random_direction(root());
    std::mt19937 random_lane_number(root());
    std::mt19937 random_driver_type(root());
    std::mt19937 random_vehicle_type(root());

    std::uniform_int_distribution<> path_distribution(1, 3);
    std::uniform_int_distribution<> direction_distribution(0, 100);
    std::uniform_int_distribution<> driver_type_distribution(1, 3);
    std::uniform_int_distribution<> vehicle_type_distribution(0, 100);

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

    uint8 north_range = simulation_params.north_spawn_probability;
    uint8 south_range = north_range + simulation_params.south_spawn_probability;
    uint8 east_range = south_range + simulation_params.east_spawn_probability;
    uint8 west_range = east_range + simulation_params.west_spawn_probability;
    switch_placeholder = direction_distribution(random_direction);
    if(switch_placeholder < north_range)
    {
        vehicle_direction = NORTH;
        std::uniform_int_distribution<> lane_number_distribution(0, intersection_params.ns_number_of_entries - 1);
        lane_number = lane_number_distribution(random_lane_number) + intersection_params.ns_number_of_exits;
    }
    else if(switch_placeholder < south_range)
    {
        vehicle_direction = SOUTH;
        std::uniform_int_distribution<> lane_number_distribution(0, intersection_params.ns_number_of_entries - 1);
        lane_number = lane_number_distribution(random_lane_number) + intersection_params.ns_number_of_exits;
    }
    else if(switch_placeholder < east_range)
    {
        vehicle_direction = EAST;
        std::uniform_int_distribution<> lane_number_distribution(0, intersection_params.ew_number_of_entries - 1);
        lane_number = lane_number_distribution(random_lane_number) + intersection_params.ew_number_of_exits;
    }
    else if(switch_placeholder <= west_range)
    {
        vehicle_direction = WEST;
        std::uniform_int_distribution<> lane_number_distribution(0, intersection_params.ew_number_of_entries - 1);
        lane_number = lane_number_distribution(random_lane_number) + intersection_params.ew_number_of_exits;
    }
    else
    {
        //likely will cause a crash in the program
        SWERRINT(switch_placeholder);
    }

    switch_placeholder = vehicle_type_distribution(random_vehicle_type);
    if(switch_placeholder < simulation_params.self_driving_vehicle_probability)
    {
        vehicle_type = SELF_DRIVING_CAR;
    }
    else
    {
        vehicle_type = CAR;
    }

    if(vehicle_type == CAR)
    {
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
    }
    else
    {
        vehicle_list[number_] = new SelfDrivingCar(number_, vehicle_path, my_intersection.getRoad(vehicle_direction)->getLane(lane_number));
    }

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

    for(uint32 j = 0; j < simulation_params.number_of_vehicles; j++)
    {
        calculateTotalAverages(vehicle_list[j]);

        switch(vehicle_list[j]->vehiclePath())
        {
            case(LEFT): calculateLeftAverages(vehicle_list[j]);
                break;
            case(STRAIGHT): calculateStraightAverages(vehicle_list[j]);
                break;
            case(RIGHT): calculateRightAverages(vehicle_list[j]);
                break;
            default:SWERRINT(vehicle_list[j]->vehiclePath());
        }
        
    }
    for(uint8 i = 0; i < TOTAL_AVERAGES; i++)
    {
        if(i == TIME_BETWEEN_SPAWNS)
        {
            averages[i] /= my_vehiclesMade;
        }
        else
        {  
            averages[i] /= my_vehiclesMade;
            if(my_selfDrivingVehiclesMade > 0)
            {
                self_driving_averages[i] /= my_selfDrivingVehiclesMade;
            }
            human_driving_averages[i] /= (my_vehiclesMade - my_selfDrivingVehiclesMade);

            if(my_leftVehiclesMade > 0)
            {
                left_averages[i] /= my_leftVehiclesMade;
                if(my_sdvLeftVehiclesMade > 0)
                {
                    sdv_left_averages[i] /= my_sdvLeftVehiclesMade;
                }
                hd_left_averages[i] /= (my_leftVehiclesMade - my_sdvLeftVehiclesMade);
            }
                
            if(my_straightVehiclesMade > 0)
            {
                straight_averages[i] /= my_straightVehiclesMade;
                if(my_sdvStraightVehiclesMade > 0)
                {
                    sdv_straight_averages[i] /= my_sdvStraightVehiclesMade;
                }
                hd_straight_averages[i] /= (my_straightVehiclesMade - my_sdvStraightVehiclesMade);
            }

            if(my_rightVehiclesMade > 0)
            {
                right_averages[i] /= my_rightVehiclesMade;
                if(my_sdvRightVehiclesMade > 0)
                {     
                    sdv_right_averages[i] /= my_sdvRightVehiclesMade;
                }
                hd_right_averages[i] /= (my_rightVehiclesMade - my_sdvRightVehiclesMade);
            }
        }
    }
    
}

void Simulation::calculateTotalAverages(Vehicle* vehicle_)
{
    if(vehicle_->vehicleType() == SELF_DRIVING_CAR)
    {
        my_selfDrivingVehiclesMade++;
    }
    for(uint8 i = 0; i < TOTAL_AVERAGES; i++)
    {
        switch(i)
        {
            case(TIME_THROUGH_INTERSECTION): 
            {
                averages[i] += vehicle_->totalTime();
                if(vehicle_->vehicleType() == CAR)
                {
                    human_driving_averages[i] += vehicle_->totalTime();
                }
                else
                {
                    self_driving_averages[i] += vehicle_->totalTime();
                }
            }
                break;
            case(TIME_IN_INTERSECTION):
            { 
                averages[i] += vehicle_->timeInIntersection();
                if(vehicle_->vehicleType() == CAR)
                {
                    human_driving_averages[i] += vehicle_->timeInIntersection();
                }
                else
                {
                    self_driving_averages[i] += vehicle_->timeInIntersection();
                }
            }
                break;
            case(TIME_AT_MAX_SPEED): 
            {
                averages[i] += vehicle_->timeAtMaxSpeed();
                if(vehicle_->vehicleType() == CAR)
                {
                    human_driving_averages[i] += vehicle_->timeAtMaxSpeed();
                }
                else
                {
                    self_driving_averages[i] += vehicle_->timeAtMaxSpeed();
                }
            }
                break;
            case(TIME_STOPPED): 
            {
                averages[i] += vehicle_->timeStopped();
                if(vehicle_->vehicleType() == CAR)
                {
                    human_driving_averages[i] += vehicle_->timeStopped();
                }
                else
                {
                    self_driving_averages[i] += vehicle_->timeStopped();
                }
            }
                break;
            case(TIME_BETWEEN_SPAWNS): continue; //already calculated
                break;
            case(FUEL_CONSUMPTION): 
            {
                averages[i] += vehicle_->fuelConsumed();
                if(vehicle_->vehicleType() == CAR)
                {
                    human_driving_averages[i] += vehicle_->fuelConsumed();
                }
                else
                {
                    self_driving_averages[i] += vehicle_->fuelConsumed();
                }
            }
                break;
            case(CO2_EMISSIONS):
            {
                averages[i] += vehicle_->emissions();
                if(vehicle_->vehicleType() == CAR)
                {
                    human_driving_averages[i] += vehicle_->emissions();
                }
                else
                {
                    self_driving_averages[i] += vehicle_->emissions();
                }
            }
                break;
            default: SWERRINT(i);
        }
    }
}

void Simulation::calculateLeftAverages(Vehicle* vehicle_)
{
    my_leftVehiclesMade++;
    if(vehicle_->vehicleType() == SELF_DRIVING_CAR)
    {
        my_sdvLeftVehiclesMade++;
    }
    for(uint8 i = 0; i < TOTAL_AVERAGES; i++)
    {
        switch(i)
        {
            case(TIME_THROUGH_INTERSECTION): 
            {
                left_averages[i] += vehicle_->totalTime();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_left_averages[i] += vehicle_->totalTime();
                }
                else
                {
                    sdv_left_averages[i] += vehicle_->totalTime();
                }
            }
                break;
            case(TIME_IN_INTERSECTION):
            { 
                left_averages[i] += vehicle_->timeInIntersection();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_left_averages[i] += vehicle_->timeInIntersection();
                }
                else
                {
                    sdv_left_averages[i] += vehicle_->timeInIntersection();
                }
            }
                break;
            case(TIME_AT_MAX_SPEED): 
            {
                left_averages[i] += vehicle_->timeAtMaxSpeed();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_left_averages[i] += vehicle_->timeAtMaxSpeed();
                }
                else
                {
                    sdv_left_averages[i] += vehicle_->timeAtMaxSpeed();
                }
            }
                break;
            case(TIME_STOPPED): 
            {
                left_averages[i] += vehicle_->timeStopped();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_left_averages[i] += vehicle_->timeStopped();
                }
                else
                {
                    sdv_left_averages[i] += vehicle_->timeStopped();
                }
            }
                break;
            case(TIME_BETWEEN_SPAWNS): continue; //already calculated
                break;
            case(FUEL_CONSUMPTION): 
            {
                left_averages[i] += vehicle_->fuelConsumed();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_left_averages[i] += vehicle_->fuelConsumed();
                }
                else
                {
                    sdv_left_averages[i] += vehicle_->fuelConsumed();
                }
            }
                break;
            case(CO2_EMISSIONS):
            {
                left_averages[i] += vehicle_->emissions();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_left_averages[i] += vehicle_->emissions();
                }
                else
                {
                    sdv_left_averages[i] += vehicle_->emissions();
                }
            }
                break;
            default: SWERRINT(i);
        }
    }
}

void Simulation::calculateStraightAverages(Vehicle* vehicle_)
{
    my_straightVehiclesMade++;
    if(vehicle_->vehicleType() == SELF_DRIVING_CAR)
    {
        my_sdvStraightVehiclesMade++;
    }
    for(uint8 i = 0; i < TOTAL_AVERAGES; i++)
    {
        switch(i)
        {
            case(TIME_THROUGH_INTERSECTION): 
            {
                straight_averages[i] += vehicle_->totalTime();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_straight_averages[i] += vehicle_->totalTime();
                }
                else
                {
                    sdv_straight_averages[i] += vehicle_->totalTime();
                }
            }
                break;
            case(TIME_IN_INTERSECTION):
            { 
                straight_averages[i] += vehicle_->timeInIntersection();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_straight_averages[i] += vehicle_->timeInIntersection();
                }
                else
                {
                    sdv_straight_averages[i] += vehicle_->timeInIntersection();
                }
            }
                break;
            case(TIME_AT_MAX_SPEED): 
            {
                straight_averages[i] += vehicle_->timeAtMaxSpeed();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_straight_averages[i] += vehicle_->timeAtMaxSpeed();
                }
                else
                {
                    sdv_straight_averages[i] += vehicle_->timeAtMaxSpeed();
                }
            }
                break;
            case(TIME_STOPPED): 
            {
                straight_averages[i] += vehicle_->timeStopped();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_straight_averages[i] += vehicle_->timeStopped();
                }
                else
                {
                    sdv_straight_averages[i] += vehicle_->timeStopped();
                }
            }
                break;
            case(TIME_BETWEEN_SPAWNS): continue; //already calculated
                break;
            case(FUEL_CONSUMPTION): 
            {
                straight_averages[i] += vehicle_->fuelConsumed();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_straight_averages[i] += vehicle_->fuelConsumed();
                }
                else
                {
                    sdv_straight_averages[i] += vehicle_->fuelConsumed();
                }
            }
                break;
            case(CO2_EMISSIONS):
            {
                straight_averages[i] += vehicle_->emissions();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_straight_averages[i] += vehicle_->emissions();
                }
                else
                {
                    sdv_straight_averages[i] += vehicle_->emissions();
                }
            }
                break;
            default: SWERRINT(i);
        }
    }
}

void Simulation::calculateRightAverages(Vehicle* vehicle_)
{
    my_rightVehiclesMade++;
    if(vehicle_->vehicleType() == SELF_DRIVING_CAR)
    {
        my_sdvRightVehiclesMade++;
    }
    for(uint8 i = 0; i < TOTAL_AVERAGES; i++)
    {
        switch(i)
        {
            case(TIME_THROUGH_INTERSECTION): 
            {
                right_averages[i] += vehicle_->totalTime();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_right_averages[i] += vehicle_->totalTime();
                }
                else
                {
                    sdv_right_averages[i] += vehicle_->totalTime();
                }
            }
                break;
            case(TIME_IN_INTERSECTION):
            { 
                right_averages[i] += vehicle_->timeInIntersection();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_right_averages[i] += vehicle_->timeInIntersection();
                }
                else
                {
                    sdv_right_averages[i] += vehicle_->timeInIntersection();
                }
            }
                break;
            case(TIME_AT_MAX_SPEED): 
            {
                right_averages[i] += vehicle_->timeAtMaxSpeed();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_right_averages[i] += vehicle_->timeAtMaxSpeed();
                }
                else
                {
                    sdv_right_averages[i] += vehicle_->timeAtMaxSpeed();
                }
            }
                break;
            case(TIME_STOPPED): 
            {
                right_averages[i] += vehicle_->timeStopped();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_right_averages[i] += vehicle_->timeStopped();
                }
                else
                {
                    sdv_right_averages[i] += vehicle_->timeStopped();
                }
            }
                break;
            case(TIME_BETWEEN_SPAWNS): continue; //already calculated
                break;
            case(FUEL_CONSUMPTION): 
            {
                right_averages[i] += vehicle_->fuelConsumed();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_right_averages[i] += vehicle_->fuelConsumed();
                }
                else
                {
                    sdv_right_averages[i] += vehicle_->fuelConsumed();
                }
            }
                break;
            case(CO2_EMISSIONS):
            {
                right_averages[i] += vehicle_->emissions();
                if(vehicle_->vehicleType() == CAR)
                {
                    hd_right_averages[i] += vehicle_->emissions();
                }
                else
                {
                    sdv_right_averages[i] += vehicle_->emissions();
                }
            }
                break;
            default: SWERRINT(i);
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
    else if (velocity_difference >= 25)
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

    results << "Total Vehicles: " << my_vehiclesMade << std::endl;
    results << "Self-Driving Vehicle Percentage: " << (float)((float)my_selfDrivingVehiclesMade / (float)my_vehiclesMade) << std::endl;
    results << "Human Driven Vehicle Percentage: " << (float)((float)(my_vehiclesMade - my_selfDrivingVehiclesMade) / (float)my_vehiclesMade) << std::endl;
    results << std::endl;
    results << "Left Vehicle Percentage: " << (float)((float)my_leftVehiclesMade / (float)my_vehiclesMade) << "\t";
    if(my_leftVehiclesMade > 0)
    {
        results << (float)((float)my_sdvLeftVehiclesMade / (float)my_leftVehiclesMade) << "\t";
        results << (float)((float)((float)my_leftVehiclesMade - (float)my_sdvLeftVehiclesMade) / (float)my_leftVehiclesMade) << std::endl;
    } 
    else
    {
        results << std::endl;
    }

    results << "Straight Vehicle Percentage: " << (float)((float)my_straightVehiclesMade / (float)my_vehiclesMade) << "\t";
    if(my_straightVehiclesMade > 0)
    {
        results << (float)((float)my_sdvStraightVehiclesMade / (float)my_straightVehiclesMade) << "\t";
        results << (float)(((float)my_straightVehiclesMade - (float)my_sdvStraightVehiclesMade) / (float)my_straightVehiclesMade) << std::endl;
    } 
    else
    {
        results << std::endl;
    }

    results << "Right Vehicle Percentage: " << (float)((float)my_rightVehiclesMade / (float)my_vehiclesMade) << "\t";
    if(my_rightVehiclesMade > 0)
    {
        results << (float)((float)my_sdvRightVehiclesMade / (float)my_rightVehiclesMade) << "\t";
        results << (float)((float)((float)my_rightVehiclesMade - (float)my_sdvRightVehiclesMade) / (float)my_rightVehiclesMade) << std::endl;
    } 
    else
    {
        results << std::endl;
    }

    results << std::endl << std::endl;

    for(uint8 i = 0; i < TOTAL_AVERAGES; i++)
    {
        results << AVERAGE_STR[i] << " " << AVERAGE_UNITS_STR[i] << ":\t" << averages[i] << "\t";
        if(i != TIME_BETWEEN_SPAWNS)
        {
            if(my_selfDrivingVehiclesMade > 0)
            {
                if(my_selfDrivingVehiclesMade == my_vehiclesMade)
                {
                    results << self_driving_averages[i] << "\t" << NOT_APPLICABLE << "\t";
                }
                else
                {
                    results << self_driving_averages[i] << "\t" << human_driving_averages[i] << "\t";
                }
            }
            else
            {
                results << NOT_APPLICABLE << "\t" << human_driving_averages[i] << "\t";
            }
            

            if(my_leftVehiclesMade > 0)
            {
                if(my_leftVehiclesMade == my_sdvLeftVehiclesMade)
                {
                    results << left_averages[i] << "\t" << sdv_left_averages[i] << "\t" << NOT_APPLICABLE << "\t";
                }
                else if(my_sdvLeftVehiclesMade > 0)
                {
                    results << left_averages[i] << "\t" << sdv_left_averages[i] << "\t" << hd_left_averages[i] << "\t";
                }
                else
                {
                    results << left_averages[i] << "\t" << NOT_APPLICABLE << "\t" << hd_left_averages[i] << "\t";
                }
            }
            else
            {
                results << NOT_APPLICABLE << "\t" << NOT_APPLICABLE << "\t" << NOT_APPLICABLE << "\t";
            }

            if(my_straightVehiclesMade > 0)
            {   
                if(my_straightVehiclesMade == my_sdvStraightVehiclesMade)
                {
                    results << straight_averages[i] << "\t" << sdv_straight_averages[i] << "\t" << NOT_APPLICABLE << "\t";
                }
                else if(my_sdvStraightVehiclesMade > 0)
                {
                    results << straight_averages[i] << "\t" << sdv_straight_averages[i] << "\t" << hd_straight_averages[i] << "\t";
                }
                else
                {
                    results << straight_averages[i] << "\t" << NOT_APPLICABLE << "\t" << hd_straight_averages[i] << "\t";
                }
            }
            else
            {
                results << NOT_APPLICABLE << "\t" << NOT_APPLICABLE << "\t" << NOT_APPLICABLE << "\t";
            }

            if(my_rightVehiclesMade > 0)
            {
                if(my_rightVehiclesMade == my_sdvRightVehiclesMade)
                {
                    results << right_averages[i] << "\t" << sdv_right_averages[i] << "\t" << NOT_APPLICABLE << "\t";
                }
                else if(my_sdvRightVehiclesMade > 0)
                {
                    results << right_averages[i] << "\t" << sdv_right_averages[i] << "\t" << hd_right_averages[i] << "\t";
                }
                else
                {
                    results << right_averages[i] << "\t" << NOT_APPLICABLE << "\t" << hd_right_averages[i] << "\t";
                }
            }
            else
            {
                results << NOT_APPLICABLE << "\t" << NOT_APPLICABLE << "\t" << NOT_APPLICABLE << "\t";
            }
        } 
        else
        {
            //sdv total and hd total
            results << NOT_APPLICABLE << "\t" << NOT_APPLICABLE << "\t";
            //lefts
            results << NOT_APPLICABLE << "\t" << NOT_APPLICABLE << "\t" << NOT_APPLICABLE << "\t";
            //straights
            results << NOT_APPLICABLE << "\t" << NOT_APPLICABLE << "\t" << NOT_APPLICABLE << "\t";
            //rights
            results << NOT_APPLICABLE << "\t" << NOT_APPLICABLE << "\t" << NOT_APPLICABLE << "\t";
        }
        results << std::endl;
    }
    results << std::endl << std::endl;
    results << "************************" << std::endl;
    results << "Fuel Economy: " << GRADE_STR[fuelConsumptionGrade(human_driving_averages[FUEL_CONSUMPTION])] << std::endl;
    results << "************************" << std::endl;
    results.close();
}

void Simulation::printResultsForPython()
{
    python_results.open("./Output/PythonResults.txt");
    python_results << elapsed_time << std::endl;
    python_results << (float)((float)my_selfDrivingVehiclesMade / (float)my_vehiclesMade) << std::endl;
    python_results << (float)((float)(my_vehiclesMade - my_selfDrivingVehiclesMade) / (float)my_vehiclesMade) << std::endl;
    python_results << (float)((float)my_leftVehiclesMade / (float)my_vehiclesMade) << std::endl;
    if(my_leftVehiclesMade > 0)
    {
        python_results << (float)((float)my_sdvLeftVehiclesMade / (float)my_leftVehiclesMade) << std::endl;
        python_results << (float)((float)((float)my_leftVehiclesMade - (float)my_sdvLeftVehiclesMade) / (float)my_leftVehiclesMade) << std::endl;
    } 
    else
    {
        python_results << 0 << std::endl;
        python_results << 0 << std::endl;
    }

    python_results << (float)((float)my_straightVehiclesMade / (float)my_vehiclesMade) << std::endl;
    if(my_straightVehiclesMade > 0)
    {
        python_results << (float)((float)my_sdvStraightVehiclesMade / (float)my_straightVehiclesMade) << std::endl;
        python_results << (float)(((float)my_straightVehiclesMade - (float)my_sdvStraightVehiclesMade) / (float)my_straightVehiclesMade) << std::endl;
    } 
    else
    {
        python_results << 0 << std::endl;
        python_results << 0 << std::endl;
    }

    python_results << (float)((float)my_rightVehiclesMade / (float)my_vehiclesMade) << std::endl;
    if(my_rightVehiclesMade > 0)
    {
        python_results << (float)((float)my_sdvRightVehiclesMade / (float)my_rightVehiclesMade) << std::endl;
        python_results << (float)((float)((float)my_rightVehiclesMade - (float)my_sdvRightVehiclesMade) / (float)my_rightVehiclesMade) << std::endl;
    } 
    else
    {
        python_results << 0 << std::endl;
        python_results << 0 << std::endl;
    }

    for(uint8 i = 0; i < TOTAL_AVERAGES; i++)
    {
        python_results << averages[i] << std::endl;
        python_results << self_driving_averages[i] << std::endl;
        python_results << human_driving_averages[i] << std::endl;
        python_results << left_averages[i] << std::endl;
        python_results << sdv_left_averages[i] << std::endl;
        python_results << hd_left_averages[i] << std::endl;
        python_results << straight_averages[i] << std::endl;
        python_results << sdv_straight_averages[i] << std::endl;
        python_results << hd_straight_averages[i] << std::endl;
        python_results << right_averages[i] << std::endl;
        python_results << sdv_right_averages[i] << std::endl;
        python_results << hd_right_averages[i] << std::endl;
    }
    python_results.close();
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
    collision << "Time In Light State: " << my_intersection.trafficLight()->timer() << "/" << my_intersection.trafficLight()->currentEventDuration() << std::endl;
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

    collision << std::endl << std::endl << std::endl;
    
    for (uint32 i = 0; i < active_vehicles.size(); i++)
    {
        if(active_vehicles[i] == first_vehicle_ || active_vehicles[i] == second_vehicle_)
        {
            continue;
        }
        else
        {
            collision << active_vehicles[i]->number() << "\t" << DRIVER_TYPE_STR[active_vehicles[i]->driverType()] << "\t";
            collision << DIRECTION_STR[active_vehicles[i]->vehicleDirection()] << "\t" << PATH_STR[active_vehicles[i]->vehiclePath()] << "\t" << VEHICLE_TYPE_STR[active_vehicles[i]->vehicleType()] << std::endl;
            collision << (int)active_vehicles[i]->currentState() << "\t" << active_vehicles[i]->currentPosition()[x] << "\t" << active_vehicles[i]->currentPosition()[y] << "\t";
            collision << active_vehicles[i]->currentVelocity()[x] << "\t" << active_vehicles[i]->currentVelocity()[y] << "\t";
            collision << active_vehicles[i]->currentAccelerationMagnitude() << "\t" << active_vehicles[i]->currentAcceleration()[x] << "\t" << active_vehicles[i]->currentAcceleration()[y] <<  std::endl;
            collision << std::endl << std::endl;
        }
    }

    collision.close();
}

void Simulation::printVehicleFailInformation(Vehicle* vehicle_)
{
    std::ofstream fail;
    std::string file_name = "./Output/FailureReport.txt";
    fail.open(file_name);
    fail << "***************" << std::endl;
    fail << "VEHICLE FAILURE" << std::endl;
    fail << "***************" << std::endl;
    fail << std::endl << std::endl;

    fail << vehicle_->number() << "\t" << DRIVER_TYPE_STR[vehicle_->driverType()] << "\t";
    fail << DIRECTION_STR[vehicle_->vehicleDirection()] << "\t" << PATH_STR[vehicle_->vehiclePath()] << "\t" << VEHICLE_TYPE_STR[vehicle_->vehicleType()] << std::endl;
    fail << (int)vehicle_->currentState() << "\t" << vehicle_->currentPosition()[x] << "\t" << vehicle_->currentPosition()[y] << "\t";
    fail << vehicle_->currentVelocity()[x] << "\t" << vehicle_->currentVelocity()[y] << "\t";
    fail << vehicle_->currentAccelerationMagnitude() << "\t" << vehicle_->currentAcceleration()[x] << "\t" << vehicle_->currentAcceleration()[y] << "\t";
    for (uint8 k = 0; k < TOTAL_POINTS; k++)
    {
        fail << vehicle_->exteriorPosition(k)[x] << "\t" << vehicle_->exteriorPosition(k)[y] << "\t";
    }
    fail << std::endl;

    fail << std::endl << std::endl << std::endl;

    fail << "Current Light State: " << LIGHT_EVENT_STATE_STR[my_intersection.trafficLight()->currentEvent()] << " (" << my_intersection.trafficLight()->state() <<")" << std::endl;
    fail << "Time In Light State: " << my_intersection.trafficLight()->timer() << "/" << my_intersection.trafficLight()->currentEventDuration() << std::endl << std::endl;

    fail << "Number of Active Vehicles: " << active_vehicles.size() << std::endl;
    fail << "Number of Vehicles Made: " << my_vehiclesMade << std::endl << std::endl;

    fail << "Elapesed Time: " << elapsed_time << std::endl;

    fail << "Vehicle " << vehicle_->number() << " Times: " << vehicle_->totalTime() << "\t" << vehicle_->timeInIntersection() << "\t";
    fail << vehicle_->timeAtMaxSpeed() << "\t" << vehicle_->timeStopped() << "\t" << std::endl;
    
    fail << "Brakelights Active: " << vehicle_->brakeLights() << std::endl;
    fail << "Blinkers Active: " << vehicle_->blinker(0) << " " << vehicle_->blinker(1) << std::endl;

    fail << std::endl << std::endl << std::endl;

    fail << "*************************" << std::endl;
    fail << "OTHER VEHICLE INFORMATION" << std::endl;
    fail << "*************************" << std::endl;
    fail << std::endl << std::endl;

    for (uint32 i = 0; i < active_vehicles.size(); i++)
    {
        if(active_vehicles[i] == vehicle_)
        {
            continue;
        }
        else
        {
            fail << active_vehicles[i]->number() << "\t" << DRIVER_TYPE_STR[active_vehicles[i]->driverType()] << "\t";
            fail << DIRECTION_STR[active_vehicles[i]->vehicleDirection()] << "\t" << PATH_STR[active_vehicles[i]->vehiclePath()] << "\t" << VEHICLE_TYPE_STR[active_vehicles[i]->vehicleType()] << std::endl;
            fail << (int)active_vehicles[i]->currentState() << "\t" << active_vehicles[i]->currentPosition()[x] << "\t" << active_vehicles[i]->currentPosition()[y] << "\t";
            fail << active_vehicles[i]->currentVelocity()[x] << "\t" << active_vehicles[i]->currentVelocity()[y] << "\t";
            fail << active_vehicles[i]->currentAccelerationMagnitude() << "\t" << active_vehicles[i]->currentAcceleration()[x] << "\t" << active_vehicles[i]->currentAcceleration()[y] <<  std::endl;
            fail << std::endl << std::endl;
        }
    }

    fail.close();
}