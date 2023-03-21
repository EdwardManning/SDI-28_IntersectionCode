#include "./SelfDrivingCar.h"

SelfDrivingCar::SelfDrivingCar(uint32 number_, path path_, Lane* lane_)
{
    my_number = number_;
    my_vehicleType = SELF_DRIVING_CAR;
    my_driver = new SelfDriver();
    my_name = VEHICLE_TYPE_STR[my_vehicleType] + " " + std::to_string(my_number);
    my_maxSpeed = lane_->speedLimit();
    my_timeInIntersection = 0;
    my_totalTime = 0;
    my_stopTime = 0;
    my_timeAtMaxSpeed = 0;
    my_completionStatus = false;
    my_maxDeceleration = 7;
    my_accelerationMagnitude = 0;
    my_currentSeparation = -1;

    my_brakeLights = 0;
    my_blinker[0] = 0;
    my_blinker[1] = 0;

    //my_idleFuelConsumption = 2500 * 3600; //Wh
    my_idleFuelConsumption = 0; //Wh
    my_fuelConsumptionAtVelocity = 0.2; //Wh/m
    my_totalFuelConsumption = 0;
    my_totalEmissions = 0;

    my_stoplineCenter = -1;

    my_currentPosition[x] = lane_->startingPosition()[x];
    my_currentPosition[y] = lane_->startingPosition()[y];

    my_currentVelocity[x] = my_maxSpeed * lane_->unitVector()[x];
    my_currentVelocity[y] = my_maxSpeed * lane_->unitVector()[y];

    my_unitVector[x] = lane_->unitVector()[x];
    my_unitVector[y] = lane_->unitVector()[y];

    my_currentAcceleration[x] = 0;
    my_currentAcceleration[y] = 0;

    my_path = path_;
    my_state = DRIVING;

    my_direction = lane_->laneDirection();
    my_laneNumber = lane_->number();

    my_runningLight = false;

    if(my_direction == NORTH || my_direction ==SOUTH)
    {
        my_stopline = lane_->endingPosition()[y];
        my_dot = y;
    }
    else
    {
        my_stopline = lane_->endingPosition()[x];
        my_dot = x;
    }

    correctLane(lane_, true);

    if(my_path != STRAIGHT)
    {
        switch(my_direction)
        {
            case(NORTH):
            {
                if(my_path == LEFT)
                {
                    my_modifier[x] = positive_sin;
                    my_modifier[y] = positive_cos;

                    my_turnRadius[x] = intersection_params.ns_left_turn_radii[x];
                    my_turnRadius[y] = intersection_params.ns_left_turn_radii[y];
                }
                else if (my_path == RIGHT)
                {
                    my_modifier[x] = negative_sin;
                    my_modifier[y] = positive_cos;

                    my_turnRadius[x] = intersection_params.ns_right_turn_radii[x];
                    my_turnRadius[y] = intersection_params.ns_right_turn_radii[y];
                }
                else
                {
                    SWERRINT(my_path);
                }
                
            }
                break;
            case(SOUTH):
            {
                if(my_path == LEFT)
                {
                    my_modifier[x] = negative_sin;
                    my_modifier[y] = negative_cos;

                    my_turnRadius[x] = intersection_params.ns_left_turn_radii[x];
                    my_turnRadius[y] = intersection_params.ns_left_turn_radii[y];
                }
                else if (my_path == RIGHT)
                {
                    my_modifier[x] = negative_sin;
                    my_modifier[y] = negative_cos;

                    my_turnRadius[x] = intersection_params.ns_right_turn_radii[x];
                    my_turnRadius[y] = intersection_params.ns_right_turn_radii[y];
                }
                else
                {
                    SWERRINT(my_path);
                }
            }
                break;
            case(EAST):
            {
                if(my_path == LEFT)
                {
                    my_modifier[x] = negative_cos;
                    my_modifier[y] = positive_sin;

                    my_turnRadius[x] = intersection_params.ew_left_turn_radii[x];
                    my_turnRadius[y] = intersection_params.ew_left_turn_radii[y];
                }
                else if (my_path == RIGHT)
                {
                    my_modifier[x] = negative_cos;
                    my_modifier[y] = positive_sin;

                    my_turnRadius[x] = intersection_params.ew_right_turn_radii[x];
                    my_turnRadius[y] = intersection_params.ew_right_turn_radii[y];
                }
                else
                {
                    SWERRINT(my_path);
                }
            }
                break;
            case(WEST):
            {
                if(my_path == LEFT)
                {
                    my_modifier[x] = positive_cos;
                    my_modifier[y] = negative_sin;

                    my_turnRadius[x] = intersection_params.ew_left_turn_radii[x];
                    my_turnRadius[y] = intersection_params.ew_left_turn_radii[y];
                }
                else if (my_path == RIGHT)
                {
                    my_modifier[x] = positive_cos;
                    my_modifier[y] = positive_sin;

                    my_turnRadius[x] = intersection_params.ew_right_turn_radii[x];
                    my_turnRadius[y] = intersection_params.ew_right_turn_radii[y];
                }
                else
                {
                    SWERRINT(my_path);
                }
            }
                break;
            default: SWERRINT(my_direction);
        }
    }
    draw(true);
    if (simulation_params.print_vehicle_info)
    {
        std::string filename = "./Output/VehicleOutput/Vehicle" + std::to_string((int)my_number) + ".txt";
        info.open(filename);
        printStartingInformation();
        printStep();
    }
}

SelfDrivingCar::~SelfDrivingCar()
{
    if (simulation_params.print_vehicle_info && info.is_open())
    {
        info.close();
    }
}

void SelfDrivingCar::consumeFuel()
{
    float velocity_magnitude = MAGNITUDE(my_currentVelocity[x], my_currentVelocity[y]);
    if(velocity_magnitude == 0 && (my_currentVelocity[x] != 0 || my_currentVelocity[y] != 0))
    {
        SWERRFLOAT(my_currentVelocity[x] != 0 ? my_currentVelocity[x] : my_currentVelocity[y]);
    }
    //28% of Ontario's power grid is natural gas (fossil fuel based)
    //1Wh of electricity takes 92.67316535mL of natural gas (https://www.ocean.washington.edu/courses/envir215/energynumbers.pdf)
    float electricity_to_gas_modifier = 0.28 * 92.67317;
    float gas_to_kgCO2 = 1.94583997 / 1000000; //kg/ml
    float fuel_used_at_step = electricity_to_gas_modifier * (((my_fuelConsumptionAtVelocity * velocity_magnitude) + my_idleFuelConsumption) * simulation_params.time_step);
    my_totalFuelConsumption += fuel_used_at_step;
    my_totalEmissions += gas_to_kgCO2 * fuel_used_at_step;
}