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
    vehicle_list = new Vehicle*[simulation_params.number_of_vehicles];
    
    generateVehicle(my_vehiclesMade);
    my_vehiclesMade++;
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
    printResults();
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
    float spawn_countdown = 4;
    //debugIntersection();
    while((!completionCheck() || (my_vehiclesMade != simulation_params.number_of_vehicles)) && elapsed_time < 300)
    {
        if (vehicle_list[0]->vehicleType() == CAR)
        {
            driverPerformActions(vehicle_list[0]);
        }
        else
        {
            //will eventually be used with vehiclePerformActions
            //for autonomous vehicles
            SWERRINT(vehicle_list[0]->vehicleType());
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
        spawn_countdown -= simulation_params.time_step;
        if (spawn_countdown <= 0 && my_vehiclesMade < simulation_params.number_of_vehicles)
        {
            spawn_countdown = 4;
            generateVehicle(my_vehiclesMade);
            my_vehiclesMade++;
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
    for(uint32 i = 0; i < simulation_params.number_of_vehicles; i++)
    {
        if (!vehicleCompleted(vehicle_list[i]))
        {
            return false;
        }
    }
    return true;
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
            SWERRINT(vehicle_->currentState());
        }
        //if it is in the lane
        if(my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->inLane(vehicle_->number()))
        {
            my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->removeFromLane(vehicle_->number());
        }
        else
        {
            SWERRINT(vehicle_->currentState());
        }
        //if it is in the road
        if(my_intersection.getRoad(vehicle_->vehicleDirection())->inRoad(vehicle_->number()))
        {
            my_intersection.getRoad(vehicle_->vehicleDirection())->removeFromRoad(vehicle_->number());
        }
        else
        {
            SWERRINT(vehicle_->currentState());
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
    
    if(!(vehicle_->currentState() & THROUGH_INTERSECTION) &&
       !(vehicle_->currentState() & IN_INTERSECTION))
    {
        if (light_change_occured)
        {
            if(vehicle_->lightChange(my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection())))
            {
                //driving check is necessary because it could be tripped if stopped
                if(!(vehicle_->currentState() & DECELERATING) &&
                   (vehicle_->currentState() & DRIVING))
                {      
                    vehicle_->accelerate(STOP);
                    if (vehicle_->currentAccelerationMagnitude() > 0)
                    {
                        changeState(vehicle_, DECELERATING, ADD);
                        if (vehicle_->currentState() & ACCELERATING)
                        {
                            changeState(vehicle_, ACCELERATING, REMOVE);
                        }
                    }
                    else
                    {
                        SWERRFLOAT(vehicle_->currentAccelerationMagnitude());
                    }
                }
            }
            else
            {
                if (MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]) < vehicle_->maxSpeed())
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
                            }
                        }
                        else
                        {
                            SWERRFLOAT(vehicle_->currentAccelerationMagnitude());
                        }
                    }
                }
            }
        }
        else //it could already be red or yellow
        {
            if(my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()) == YELLOW)
            {
                if (vehicle_->yellowLightAnalysis())
                {
                    //driving check is necessary because it could be tripped if stopped
                    if(!(vehicle_->currentState() & DECELERATING) && 
                       (vehicle_->currentState() & DRIVING))
                    {      
                        vehicle_->accelerate(STOP);
                        if (vehicle_->currentAccelerationMagnitude() > 0)
                        {
                            changeState(vehicle_, DECELERATING, ADD);
                            if (vehicle_->currentState() & ACCELERATING)
                            {
                                changeState(vehicle_, ACCELERATING, REMOVE);
                            }
                        }
                        else
                        {
                        SWERRFLOAT(vehicle_->currentAccelerationMagnitude());
                        }
                    }
                }
            }
            else if(my_intersection.trafficLight()->currentLightColour(vehicle_->vehicleDirection()) == RED)
            {
                if (vehicle_->redLightAnalysis())
                {
                    //driving check is necessary because it could be tripped if stopped
                    if(!(vehicle_->currentState() & DECELERATING) &&
                       (vehicle_->currentState() & DRIVING))
                    {      
                        vehicle_->accelerate(STOP);
                        if (vehicle_->currentAccelerationMagnitude() > 0)
                        {
                            changeState(vehicle_, DECELERATING, ADD);
                            if (vehicle_->currentState() & ACCELERATING)
                            {
                                changeState(vehicle_, ACCELERATING, REMOVE);
                            }
                        }
                        else
                        {
                            SWERRFLOAT(vehicle_->currentAccelerationMagnitude());
                        }
                    }
                }
            }
        }
    }
    else //through intersection, no red or yellow lights
    {
        if (MAGNITUDE(vehicle_->currentVelocity()[x], vehicle_->currentVelocity()[y]) < vehicle_->maxSpeed())
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
                    }
                }
                else
                {
                    SWERRFLOAT(vehicle_->currentAccelerationMagnitude());
                }
            }
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
                if (simulation_params.print_simulation_events)
                {
                    printLaneChange(vehicle_, new_lane);
                }
                vehicle_->setLane(new_lane);
                if(vehicle_->correctLane(my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())))
                {
                    changeState(vehicle_, CORRECT_LANE, ADD);
                    changeState(vehicle_, CHANGING_LANES, REMOVE);
                }
                vehicle_->stopLaneChange();
            }
        }
        else
        {
            changeState(vehicle_, CHANGING_LANES, ADD);
            vehicle_->changeLane(lane_change_direction);
        }
    }
    if ((vehicle_->currentState() & ACCELERATING) | (vehicle_->currentState() & DECELERATING))
    {
        if(vehicle_->accelerate())
        {
            if (vehicle_->currentState() & DECELERATING)
            {
                changeState(vehicle_, DECELERATING, REMOVE);
                changeState(vehicle_, DRIVING, REMOVE);
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
    //lane should have changed by now
    if(!my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->inLane(vehicle_->number()))
    {
        my_intersection.getRoad(vehicle_->vehicleDirection())->getLane(vehicle_->laneNumber())->addToLane(vehicle_->number());
    }
    //if it is a lane change and not being set because of exiting the intersection this will be false
    //because it does not need to change roads
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
    }
    return return_value;
}

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
    std::cout << number_ << std::endl;
    vehicle_list[number_] = new Car(number_, vehicle_path, my_intersection.getRoad(vehicle_direction)->getLane(lane_number), driver_type);

    if(simulation_params.print_simulation_events)
    {
        printVehicleArrival(vehicle_list[number_]);
    }
    addToActiveVehicles(vehicle_list[number_]);
    my_intersection.getRoad(vehicle_list[number_]->vehicleDirection())->addToRoad(number_);
    my_intersection.getRoad(vehicle_list[number_]->vehicleDirection())->getLane(vehicle_list[number_]->laneNumber())->addToLane(number_);
}

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

    results.open("./Output/Results.txt");
    results << vehicle_list[0]->totalTime() << std::endl;
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