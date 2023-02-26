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
    while(!completionCheck())
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
    if(my_vehiclesMade < simulation_params.number_of_vehicles)
    {
        return false;
    }
    else
    {
        bool return_value = true;
        for(uint32 i = 0; i < simulation_params.number_of_vehicles; i++)
        {
            if (!vehicleCompleted(vehicle_list[i]))
            {
                return_value = false;
            }
        }
        if(return_value)
        {
            printResults();
        }
        return return_value;
    }
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
    
    determineAcceleration(vehicle_);
    
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
void Simulation::determineAcceleration(Vehicle* vehicle_)
{
    if(!closeVehicleDecelerationRequired(vehicle_))
    {
        if(!laneChangeDecelerationRequired(vehicle_))
        {
            if(!lightChangeDecelerationRequired(vehicle_))
            {
                if(!lightColourDecelerationRequired(vehicle_))
                {
                    if(!maxSpeedDecelerationRequired(vehicle_))
                    {
                        //if there is no need to decelerate we can accelerate if necessary
                        if(MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y] < vehicle_->maxSpeed()))
                        {
                            startAcceleration(vehicle_, vehicle_->maxSpeed());
                        }
                    }
                }
            }
        }
        else
        {
            if(light_change_occured)
            {
                if((vehicle_->yellowLightAnalysis(vehicle_->currentAccelerationMagnitude()) &&
                   my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()) == YELLOW) ||
                   (vehicle_->redLightAnalysis(vehicle_->currentAccelerationMagnitude()) &&
                   my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()) == RED))
                {
                    //if the potential required deceleration due to the light changing is greater
                    //than the deceleration due to vehicles around, vehicle must stop faster
                    //this will not necessarily increase the deceleration but it will check if it 
                    //is necessary
                    lightChangeDecelerationRequired(vehicle_);
                }
            }
        }
    }
    else
    {
        if(light_change_occured)
        {
            if((vehicle_->yellowLightAnalysis(vehicle_->currentAccelerationMagnitude()) &&
               my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()) == YELLOW) ||
               (vehicle_->redLightAnalysis(vehicle_->currentAccelerationMagnitude()) &&
               my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()) == RED))
            {
                //if the potential required deceleration due to the light changing is greater
                //than the deceleration due to vehicles around, vehicle must stop faster
                //this will not necessarily increase the deceleration but it will check if it 
                //is necessary
                lightChangeDecelerationRequired(vehicle_);
            }
        }
    }
    return;
}

bool Simulation::closeVehicleDecelerationRequired(Vehicle* vehicle_)
{
    if(vehicle_->currentState() & THROUGH_INTERSECTION)
    {
        return postIntersectionCloseDecelerationRequired(vehicle_);
    }
    else if(vehicle_->currentState() & IN_INTERSECTION)
    {
        return inIntersectionCloseDecelerationRequired(vehicle_);
    }
    else //haven't entered the interesection yet
    {
        return preIntersectionCloseDecelerationRequired(vehicle_);
    }
}

bool Simulation::preIntersectionCloseDecelerationRequired(Vehicle* vehicle_)
{
    if(vehicleAhead(vehicle_, false))
    {
        Vehicle* vehicle_ahead = whichVehicleAhead(vehicle_, false);
        if(vehicle_->number() != vehicle_ahead->number())
        {
            float current_separation = vehicleSeparation(vehicle_, vehicle_ahead);
            if(vehicle_->currentSeparation() == -1)
            {
                vehicle_->setCurrentSeparation(current_separation);
            }

            if(current_separation < vehicle_->minimumFollowingDistance())
            {
                
                if(!(vehicle_->currentState() & DECELERATING))
                {
                    float target_speed = 0.75 * MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]);
                    startDeceleration(vehicle_, target_speed);
                }
                else
                {
                    //increase deceleration by 0.5m/s^2
                    vehicle_->requestAccelerationAdjustment(0.5);
                }
                vehicle_->setCurrentSeparation(current_separation);
                return true;
            }
            else
            {
                if(current_separation < vehicle_->currentSeparation() ||
                   vehicle_ahead->brakeLights())
                {
                    
                    if(!(vehicle_->currentState() & DECELERATING))
                    {
                        float target_speed = 0.95 * MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]);
                        startDeceleration(vehicle_, target_speed);
                    }
                    else
                    {
                        //increase deceleration by 0.5m/s^2
                        vehicle_->requestAccelerationAdjustment(0.5);
                    }
                    vehicle_->setCurrentSeparation(current_separation);
                    return true;
                }
            }
            vehicle_->setCurrentSeparation(current_separation);
        }
        else
        {
            SWERRINT(vehicle_->number());
        }
    }
    if (closeLaneChangeDecelerationRequired(vehicle_))
    {
        if(!(vehicle_->currentState() & DECELERATING))
        {
            float target_speed = 0.85 * MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]);
            startDeceleration(vehicle_, target_speed);
        }
        else
        {
            //increase deceleration by 0.5m/s^2
            vehicle_->requestAccelerationAdjustment(0.5);
        }
        return true;
    }
    return false;
}

bool Simulation::closeLaneChangeDecelerationRequired(Vehicle* vehicle_)
{
    path current_lane_path = my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->lanePath();
    switch(current_lane_path)
    {
        case(LEFT):
        {
            return (checkLaneBlinkers(vehicle_, my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber() + 1), 1));
        }
            break;
        case(RIGHT):
        {
            return (checkLaneBlinkers(vehicle_, my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber() - 1), -1));
        }
            break;
        case(STRAIGHT):
        {
            return (checkLaneBlinkers(vehicle_, my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber() + 1), 1)) ||
                   (checkLaneBlinkers(vehicle_, my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber() - 1), -1));
        }
            break;
        default: SWERRINT(current_lane_path);
    }
    return false;
}

bool Simulation::inIntersectionCloseDecelerationRequired(Vehicle* vehicle_)
{
    if(scanAhead(vehicle_))
    {
        if(!(vehicle_->currentState() & DECELERATING))
        {
            float target_speed = 0.75 * MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]);
            startDeceleration(vehicle_, target_speed);
        }
        else
        {
            float target_speed = 0.95 * MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]);
            changeDeceleration(vehicle_, target_speed);
        }
        return true;
    }

    if(vehicle_->vehiclePath() == LEFT)
    {
        if(!checkIntersectionClear(vehicle_))
        {
            startDeceleration(vehicle_, STOP);
            return true;
        }
    }
    return false;
}

bool Simulation::postIntersectionCloseDecelerationRequired(Vehicle* vehicle_)
{
    if(vehicleAhead(vehicle_, true))
    {
        Vehicle* vehicle_ahead = whichVehicleAhead(vehicle_, true);
        if(vehicle_->number() != vehicle_ahead->number())
        {
            float current_separation = vehicleSeparation(vehicle_, vehicle_ahead);
            if(vehicle_->currentSeparation() == -1)
            {
                vehicle_->setCurrentSeparation(current_separation);
            }

            if(current_separation < vehicle_->minimumFollowingDistance())
            {
                
                if(!(vehicle_->currentState() & DECELERATING))
                {
                    float target_speed = 0.75 * MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]);
                    startDeceleration(vehicle_, target_speed);
                }
                else
                {
                    //increase deceleration by 0.5m/s^2
                    vehicle_->requestAccelerationAdjustment(0.5);
                }
                vehicle_->setCurrentSeparation(current_separation);
                return true;
            }
            else
            {
                if(current_separation < vehicle_->currentSeparation() ||
                   vehicle_ahead->brakeLights())
                {
                    
                    if(!(vehicle_->currentState() & DECELERATING))
                    {
                        float target_speed = 0.95 * MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]);
                        startDeceleration(vehicle_, target_speed);
                    }
                    else
                    {
                        //increase deceleration by 0.5m/s^2
                        vehicle_->requestAccelerationAdjustment(0.5);
                    }
                    vehicle_->setCurrentSeparation(current_separation);
                    return true;
                }
            }
            vehicle_->setCurrentSeparation(current_separation);
        }
        else
        {
            SWERRINT(vehicle_->number());
        }
    }
    return false;
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
        if(!(vehicle_->currentState() & DECELERATING))
        {
            float new_speed = 0.8 * vehicle_->maxSpeed();
            startDeceleration(vehicle_, new_speed);
        }
        return true;
    }
    return false;
}

bool Simulation::lightChangeDecelerationRequired(Vehicle* vehicle_)
{
    //no lights in or through intersection
    if((vehicle_->currentState() & IN_INTERSECTION) || vehicle_->currentState() & THROUGH_INTERSECTION)
    {
        return false;
    }
    if(!light_change_occured)
    {
        return false;
    }
    //light change returns true if decelerating is required
    if(vehicle_->lightChange(my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection())))
    {
        startDeceleration(vehicle_, STOP);
        return true;
    }
    return false;
}

bool Simulation::lightColourDecelerationRequired(Vehicle* vehicle_)
{
    //no lights in or through intersection
    if((vehicle_->currentState() & IN_INTERSECTION) || vehicle_->currentState() & THROUGH_INTERSECTION)
    {
        return false;
    }

    if(my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()) == YELLOW)
    {
        //returns true if stop required at yellow light
        if(vehicle_->yellowLightAnalysis())
        {
            startDeceleration(vehicle_, STOP);
            return true;
        }
    }
    else if(my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()) == RED)
    {
        if(vehicle_->redLightAnalysis())
        {
            startDeceleration(vehicle_, STOP);
            return true;
        }
    }
    return false;
}

bool Simulation::maxSpeedDecelerationRequired(Vehicle* vehicle_)
{
    float velocity_magnitude = MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]);
    if(velocity_magnitude > vehicle_->maxSpeed())
    {  
        startDeceleration(vehicle_, vehicle_->maxSpeed());
        return true;
    }
    return false;
}

void Simulation::startAcceleration(Vehicle* vehicle_, float target_speed_)
{
    if(MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]) < target_speed_)
    {
        if (!(vehicle_->currentState() & ACCELERATING))
        {
            vehicle_->accelerate(vehicle_->maxSpeed());
            if (vehicle_->currentAccelerationMagnitude() > 0)
            {
                changeState(vehicle_, ACCELERATING, ADD);
                if (!(vehicle_->currentState() & DRIVING))
                {
                    changeState(vehicle_, DRIVING, ADD);
                }
                if (vehicle_->currentState() & DECELERATING)
                {
                    changeState(vehicle_, DECELERATING, REMOVE);
                    vehicle_->toggleBrakeLights(OFF);
                }
            }
            else
            {
                SWERRFLOAT(vehicle_->currentAccelerationMagnitude());
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

bool Simulation::checkLaneBlinkers(Vehicle* vehicle_, Lane* lane_, int8 direction_)
{
    if(lane_->numberOfVehicles() < 1)
    {
        return false;
    }
    else
    {
        bool blinker_direction = direction_ > 0;
        bool dot = maxComponent<float>(vehicle_->unitVector()[x], vehicle_->unitVector()[y]);
        int8 modifier = vehicle_->unitVector()[dot];
        for(uint32 i = 0; i < lane_->numberOfVehicles(); i++)
        {
            if(vehicle_list[lane_->vehicleAtIndex(i)]->blinker(blinker_direction))
            {
                if (modifier > 0)
                {
                    if(vehicle_->exteriorPosition(FRONT_BUMPER)[dot] < vehicle_list[lane_->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot] &&
                       vehicle_->exteriorPosition(FRONT_BUMPER)[dot] > (vehicle_list[lane_->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot] - (1.5 * vehicle_->minimumFollowingDistance())))
                    {
                        return true;
                    }
                }
                else
                {
                    if(vehicle_->exteriorPosition(FRONT_BUMPER)[dot] > vehicle_list[lane_->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot] &&
                       vehicle_->exteriorPosition(FRONT_BUMPER)[dot] < (vehicle_list[lane_->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot] - (1.5 * vehicle_->minimumFollowingDistance())))
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }
}

bool Simulation::scanAhead(Vehicle* vehicle_)
{
    if(my_intersection.numberOfVehicles() <= 1)
    {
        return false;
    }
    float scan_distance = vehicle_->minimumFollowingDistance();
    bool dot = maxComponent<float>(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]);
    bool not_dot = !dot;
    if (vehicle_->currentVelocity()[not_dot] == 0)
    {
        for(uint32 i = 0; i < my_intersection.numberOfVehicles(); i++)
        {
            if(my_intersection.vehicleAtIndex(i) != vehicle_->number())
            {
                if(vehicle_->unitVector()[dot] > 0)
                {
                    if (vehicle_list[my_intersection.vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot]
                            <= vehicle_->exteriorPosition(FRONT_BUMPER)[dot] + scan_distance)
                    {
                        return true;
                    }
                }
                else
                {
                    if (vehicle_list[my_intersection.vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot]
                            >= vehicle_->exteriorPosition(FRONT_BUMPER)[dot] - scan_distance)
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    else
    {
        float theta = atan(vehicle_->currentVelocity()[not_dot] / vehicle_->currentVelocity()[dot]);
        float dot_scan_distance = abs(scan_distance * cos(theta));
        float not_dot_scan_distance = abs(scan_distance * sin(theta));
        if(!isPositive(vehicle_->currentVelocity()[dot]))
        {
            dot_scan_distance *= -1;
        }
        if(!isPositive(vehicle_->currentVelocity()[not_dot]))
        {
            not_dot_scan_distance *= -1;
        }
        for(uint32 i = 0; i < my_intersection.numberOfVehicles(); i++)
        {
            if(my_intersection.vehicleAtIndex(i) != vehicle_->number())
            {
                Vehicle* checked_vehicle = vehicle_list[my_intersection.vehicleAtIndex(i)];
                if(checked_vehicle->exteriorPosition(FRONT_BUMPER)[dot] > checked_vehicle->exteriorPosition(BACK_BUMPER)[dot])
                {
                    if(checked_vehicle->exteriorPosition(FRONT_BUMPER)[not_dot] > checked_vehicle->exteriorPosition(BACK_BUMPER)[not_dot])
                    {
                        if (vehicle_->exteriorPosition(FRONT_BUMPER)[dot] + dot_scan_distance < checked_vehicle->exteriorPosition(FRONT_BUMPER)[dot] &&
                            vehicle_->exteriorPosition(FRONT_BUMPER)[dot] + dot_scan_distance > checked_vehicle->exteriorPosition(BACK_BUMPER)[dot] &&
                            vehicle_->exteriorPosition(FRONT_BUMPER)[not_dot] + not_dot_scan_distance < checked_vehicle->exteriorPosition(FRONT_BUMPER)[not_dot] &&
                            vehicle_->exteriorPosition(FRONT_BUMPER)[not_dot] + not_dot_scan_distance > checked_vehicle->exteriorPosition(BACK_BUMPER)[not_dot])
                        {
                            return true;
                        }
                    }
                    else
                    {
                        if (vehicle_->exteriorPosition(FRONT_BUMPER)[dot] + dot_scan_distance < checked_vehicle->exteriorPosition(FRONT_BUMPER)[dot] &&
                            vehicle_->exteriorPosition(FRONT_BUMPER)[dot] + dot_scan_distance > checked_vehicle->exteriorPosition(BACK_BUMPER)[dot] &&
                            vehicle_->exteriorPosition(FRONT_BUMPER)[not_dot] + not_dot_scan_distance > checked_vehicle->exteriorPosition(FRONT_BUMPER)[not_dot] &&
                            vehicle_->exteriorPosition(FRONT_BUMPER)[not_dot] + not_dot_scan_distance < checked_vehicle->exteriorPosition(BACK_BUMPER)[not_dot])
                        {
                            return true;
                        }
                    }
                }
                else
                {
                    if(checked_vehicle->exteriorPosition(FRONT_BUMPER)[not_dot] > checked_vehicle->exteriorPosition(BACK_BUMPER)[not_dot])
                    {
                        if (vehicle_->exteriorPosition(FRONT_BUMPER)[dot] + dot_scan_distance > checked_vehicle->exteriorPosition(FRONT_BUMPER)[dot] &&
                            vehicle_->exteriorPosition(FRONT_BUMPER)[dot] + dot_scan_distance < checked_vehicle->exteriorPosition(BACK_BUMPER)[dot] &&
                            vehicle_->exteriorPosition(FRONT_BUMPER)[not_dot] + not_dot_scan_distance < checked_vehicle->exteriorPosition(FRONT_BUMPER)[not_dot] &&
                            vehicle_->exteriorPosition(FRONT_BUMPER)[not_dot] + not_dot_scan_distance > checked_vehicle->exteriorPosition(BACK_BUMPER)[not_dot])
                        {
                            return true;
                        }
                    }
                    else
                    {
                        if (vehicle_->exteriorPosition(FRONT_BUMPER)[dot] + dot_scan_distance > checked_vehicle->exteriorPosition(FRONT_BUMPER)[dot] &&
                            vehicle_->exteriorPosition(FRONT_BUMPER)[dot] + dot_scan_distance < checked_vehicle->exteriorPosition(BACK_BUMPER)[dot] &&
                            vehicle_->exteriorPosition(FRONT_BUMPER)[not_dot] + not_dot_scan_distance > checked_vehicle->exteriorPosition(FRONT_BUMPER)[not_dot] &&
                            vehicle_->exteriorPosition(FRONT_BUMPER)[not_dot] + not_dot_scan_distance < checked_vehicle->exteriorPosition(BACK_BUMPER)[not_dot])
                        {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }
    return false;
}

bool Simulation::checkIntersectionClear(Vehicle* vehicle_)
{
    if(my_intersection.numberOfVehicles() > 1)
    {
        for(uint32 i = 0; i < my_intersection.numberOfVehicles(); i++)
        {
            if(vehicle_->number() == my_intersection.vehicleAtIndex(i))
            {
                continue;
            }
            else
            {   
                if(conflictingPaths(vehicle_, vehicle_list[my_intersection.vehicleAtIndex(i)]))
                {   
                    bool conflicting_component = maxComponent(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]);
                    float distance_from_center;
                    if(vehicle_list[my_intersection.vehicleAtIndex(i)]->unitVector()[conflicting_component] > 0)
                    {
                        distance_from_center = intersection_params.center_coordinates[conflicting_component] - vehicle_list[my_intersection.vehicleAtIndex(i)]->currentPosition()[conflicting_component];
                    }
                    else
                    {
                        distance_from_center = vehicle_list[my_intersection.vehicleAtIndex(i)]->currentPosition()[conflicting_component] - intersection_params.center_coordinates[conflicting_component];
                    }
                    if (!(distance_from_center < 0))
                    {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

bool Simulation::conflictingPaths(Vehicle* current_vehicle_, Vehicle* test_vehicle_)
{
    if(current_vehicle_->vehicleDirection() == test_vehicle_->vehicleDirection())
    {
        return false;
    }
    else
    {
        if(current_vehicle_->vehiclePath() == LEFT)
        {
            if(test_vehicle_->vehiclePath() == STRAIGHT)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
}

bool Simulation::vehicleAhead(Vehicle* vehicle_, bool through_intersection_)
{
    direction road_direction;

    if(through_intersection_)
    {
        road_direction = my_intersection.getRoad(vehicle_->vehicleDirection())->correspondingExit(vehicle_->vehiclePath());
    }
    else
    {
        road_direction = vehicle_->vehicleDirection();
    }

    Lane* current_lane = my_intersection.getRoad(road_direction)->getLane(vehicle_->laneNumber());
    uint32 lane_number_of_vehicles = current_lane->numberOfVehicles();

    if(lane_number_of_vehicles > 1)
    {
        bool dot = maxComponent<float>(vehicle_->unitVector()[x], vehicle_->unitVector()[y]);
        int8 modifier = vehicle_->unitVector()[dot];
        for(uint32 i = 0; i < lane_number_of_vehicles; i++)
        {
            if (modifier > 0)
            {
                if(vehicle_->number() != current_lane->vehicleAtIndex(i))
                {
                    if(vehicle_->exteriorPosition(FRONT_BUMPER)[dot] < vehicle_list[current_lane->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot])
                    {
                        return true;
                    }
                }
            }
            else
            {
                if(vehicle_->number() != current_lane->vehicleAtIndex(i))
                {
                    if(vehicle_->exteriorPosition(FRONT_BUMPER)[dot] > vehicle_list[current_lane->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot])
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    else
    {
        return false;
    }
}

float Simulation::vehicleSeparation(Vehicle* current_vehicle_, Vehicle* ahead_vehicle_)
{
    bool dot = maxComponent<float>(current_vehicle_->currentVelocity()[x], current_vehicle_->currentVelocity()[y]);
    int8 modifier = current_vehicle_->unitVector()[dot];
    if(modifier > 0)
    {
        return ahead_vehicle_->exteriorPosition(BACK_BUMPER)[dot] - current_vehicle_->exteriorPosition(FRONT_BUMPER)[dot];
    }
    else
    {
        return current_vehicle_->exteriorPosition(FRONT_BUMPER)[dot] - ahead_vehicle_->exteriorPosition(BACK_BUMPER)[dot];
    }
}

Vehicle* Simulation::whichVehicleAhead(Vehicle* vehicle_, bool through_intersection_)
{
    direction road_direction;

    if(through_intersection_)
    {
        road_direction = my_intersection.getRoad(vehicle_->vehicleDirection())->correspondingExit(vehicle_->vehiclePath());
    }
    else
    {
        road_direction = vehicle_->vehicleDirection();
    }

    Lane* current_lane = my_intersection.getRoad(road_direction)->getLane(vehicle_->laneNumber());
    uint32 lane_number_of_vehicles = current_lane->numberOfVehicles();

    if(lane_number_of_vehicles > 1)
    {
        bool dot = maxComponent<float>(vehicle_->unitVector()[x], vehicle_->unitVector()[y]);
        int8 modifier = vehicle_->unitVector()[dot];
        float current_lowest_separation = 10000000000; //large number so anything will be smaller
        uint32 corresponding_index_number = -1;
        for(uint32 i = 0; i < lane_number_of_vehicles; i++)
        {
            if (modifier > 0)
            {
                if(vehicle_->number() != current_lane->vehicleAtIndex(i))
                {
                    //still necessary since there can technically ve negative separation
                    if (vehicle_->exteriorPosition(FRONT_BUMPER)[dot] < vehicle_list[current_lane->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot]) 
                    {
                        if(vehicle_list[current_lane->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot] - vehicle_->exteriorPosition(FRONT_BUMPER)[dot] < current_lowest_separation)
                        {
                            current_lowest_separation = vehicle_list[current_lane->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot] - vehicle_->exteriorPosition(FRONT_BUMPER)[dot];
                            corresponding_index_number = i;
                        }
                    }
                }
            }
            else
            {
                if(vehicle_->number() != current_lane->vehicleAtIndex(i))
                {
                    if(vehicle_->exteriorPosition(FRONT_BUMPER)[dot] > vehicle_list[current_lane->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot])
                    {
                        if (vehicle_->exteriorPosition(FRONT_BUMPER)[dot] - vehicle_list[current_lane->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot] < current_lowest_separation)
                        {
                            current_lowest_separation = vehicle_->exteriorPosition(FRONT_BUMPER)[dot] - vehicle_list[current_lane->vehicleAtIndex(i)]->exteriorPosition(BACK_BUMPER)[dot];
                            corresponding_index_number = i;
                        }
                    }
                }
            }
        }
        if(current_lowest_separation < 0)
        {
            SWERRFLOAT(corresponding_index_number);
        }
        try
        {      
            if (corresponding_index_number >= 0)
            {
                return(vehicle_list[current_lane->vehicleAtIndex(corresponding_index_number)]);
            }
            else
            {
                SWERRFLOAT(current_lowest_separation);
                throw(std::out_of_range("Out of Vehicle List Bounds"));
            }
        }
        catch (const std::out_of_range &Out_of_Range)
        {
            //hard SWERR
            SWERRINT(corresponding_index_number);
            throw;
        }
    }
    SWERRINT(lane_number_of_vehicles);
    //this should never be hit, error above should crash program beforehand
    //this is here to make compiler happy and just in case so we can plan a safety net
    return vehicle_;
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
                    std::ofstream collision;
                    std::string file_name = "./Output/CollisionInformation.txt";
                    collision.open(file_name);
                    collision << vehicle_list[i]->number() << "\t" << DRIVER_TYPE_STR[vehicle_list[i]->driverType()] << "\t";
                    collision << DIRECTION_STR[vehicle_list[i]->vehicleDirection()] << "\t" << PATH_STR[vehicle_list[i]->vehiclePath()] << "\t" << VEHICLE_TYPE_STR[vehicle_list[i]->vehicleType()] << std::endl;
                    collision << (int)vehicle_list[i]->currentState() << "\t" << vehicle_list[i]->currentPosition()[x] << "\t" << vehicle_list[i]->currentPosition()[y] << "\t";
                    collision << vehicle_list[i]->currentVelocity()[x] << "\t" << vehicle_list[i]->currentVelocity()[y] << "\t";
                    for (uint8 k = 0; k < TOTAL_POINTS; k++)
                    {
                        collision << vehicle_list[i]->exteriorPosition(k)[x] << "\t" << vehicle_list[i]->exteriorPosition(k)[y] << "\t";
                    }
                    collision << std::endl;

                    collision << vehicle_list[j]->number() << "\t" << DRIVER_TYPE_STR[vehicle_list[j]->driverType()] << "\t";
                    collision << DIRECTION_STR[vehicle_list[j]->vehicleDirection()] << "\t" << PATH_STR[vehicle_list[j]->vehiclePath()] << "\t" << VEHICLE_TYPE_STR[vehicle_list[j]->vehicleType()] << std::endl;
                    collision << (int)vehicle_list[j]->currentState() << "\t" << vehicle_list[j]->currentPosition()[x] << "\t" << vehicle_list[j]->currentPosition()[y] << "\t";
                    collision << vehicle_list[j]->currentVelocity()[x] << "\t" << vehicle_list[j]->currentVelocity()[y] << "\t";
                    for (uint8 k = 0; k < TOTAL_POINTS; k++)
                    {
                        collision << vehicle_list[j]->exteriorPosition(k)[x] << "\t" << vehicle_list[j]->exteriorPosition(k)[y] << "\t";
                    }
                    collision << std::endl;
                    collision.close();
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
    for(uint8 i = 0; i < TOTAL_AVERAGES; i++)
    {
        results << AVERAGE_STR[i] << ":\t" << averages[i] << std::endl; 
    }
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