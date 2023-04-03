from pathlib import Path
from enum import IntEnum
import subprocess
import os
import time
import matplotlib.pyplot as plt
import numpy as np

STANDARD_SIMULATION_PARAMS = [1, 25]

LINE_STR = "Line"

class AVERAGES(IntEnum):
    TIME_THROUGH_INTERSECTION = 0
    TIME_IN_INTERSECTION = 1
    TIME_AT_MAX_SPEED = 2
    TIME_STOPPED = 3
    TIME_BETWEEN_SPAWNS = 4
    FUEL_CONSUMPTION = 5
    CO2_EMISSIONS = 6
    TOTAL_AVERAGES = 7

class AVERAGE_TYPES(IntEnum):
    AVERAGES = 0
    SELF_DRIVING_AVERAGES = 1
    HUMAN_DRIVING_AVERAGES = 2
    LEFT_AVERAGES = 3
    SDV_LEFT_AVERAGES = 4
    HD_LEFT_AVERAGES = 5
    STRAIGHT_AVERAGES = 6
    SDV_STRAIGHT_AVERAGES = 7
    HD_STRAIGHT_AVERAGES = 8
    RIGHT_AVERAGES = 9
    SDV_RIGHT_AVERAGES = 10
    HD_RIGHT_AVERAGES = 11
    TOTAL_AVERAGE_TYPES = 12

class RESULTS(IntEnum):
    ELAPSED_TIME = 0
    PERCENT_SDV = 1
    PERCENT_HDV = 2
    PERCENT_LEFT_VEHICLES = 3
    PERCENT_SDV_LEFT = 4
    PERCENT_HDV_LEFT = 5
    PERCENT_STRAIGHT_VEHICLES = 6
    PERCENT_SDV_STRAIGHT = 7
    PERCENT_HDV_STRAIGHT = 8
    PERCENT_RIGHT_VEHICLES = 9
    PERCENT_SDV_RIGHT = 10
    PERCENT_HDV_RIGHT = 11
    TOTAL_RESULTS = 12

class data:
    def __init__(self, vehicle_density_, self_driving_probability_):
        self.density = vehicle_density_
        self.sdv_probability = self_driving_probability_
        self.number_of_simulations = 0
        self.number_of_collisions = 0
        self.number_of_fails = 0
        self.number_of_data_points = [[0 for x in range(AVERAGE_TYPES.TOTAL_AVERAGE_TYPES)] for y in range(AVERAGES.TOTAL_AVERAGES)]
        self.simulation_results = [ 0 for x in range(RESULTS.TOTAL_RESULTS)]
        self.simulation_averages = [[0 for x in range(AVERAGE_TYPES.TOTAL_AVERAGE_TYPES)] for y in range(AVERAGES.TOTAL_AVERAGES)]
    
    def add_results(self, result_type_, data):
        self.simulation_results[result_type_] += data
    
    def add_averages(self, average_, average_type_, data):
        self.simulation_averages[average_][average_type_] += data
        self.number_of_data_points[average_][average_type_] += 1

    def add_collision(self):
        self.number_of_collisions += 1
    
    def add_failure(self):
        self.number_of_fails += 1

    def new_simulation(self):
        self.number_of_simulations += 1
    
    def finalize_averages(self):
        for i in range(AVERAGES.TOTAL_AVERAGES):
            for j in range(AVERAGE_TYPES.TOTAL_AVERAGE_TYPES):
                if(self.number_of_data_points[i][j] > 0):
                    self.simulation_averages[i][j] /= self.number_of_data_points[i][j]
                else:
                    if(self.simulation_averages[i][j] != 0):
                        print("Non-zero average for zero data points")
                        print(str(self.density) + " " + str(self.sdv_probability))
                        print(str(i) + " " + str(j))
    
    def finalize_results(self):
        for i in range(RESULTS.TOTAL_RESULTS):
            if(self.number_of_simulations > 0):
                self.simulation_results[i] /= self.number_of_simulations
            else:
                print("Number of Simulations Error: " + str(self.number_of_simulations))
        
    def finalize(self):
        self.finalize_averages()
        self.finalize_results()

collision_text_path = Path('./Output/CollisionInformation.txt')
all_collisions_text_path = Path('./Output/AllCollisions.txt')
fail_report_text_path = Path('./Output/FailureReport.txt')
all_fail_report_text_path = Path('./Output/AllFailures.txt')
results_path = Path('./Output/PythonResults.txt')
test_output_path = Path('./Output/PythonTestOutput.txt')
second_output_path = Path('./Output/AuxPythonTestOutput.txt')
simulation_params_input = Path('./Input/SimulationParamsInput.txt')
events_path = Path('./Output/Events.txt')
formatted_results_path = Path('./Output/Results.txt')
SWERR_path = Path('./Output/SwerrList.txt')
completed_swerrs = Path('./Output/CompletionSwerrs.txt')
vehicle_outputs_path = Path('./Output/VehicleOutput')
images_output_path_str = "./Output/Images/"
images_output_path = Path(images_output_path_str)

def readData():
    total_data = []
    results = [0 for x in range(RESULTS.TOTAL_RESULTS)]
    average_values = [[0 for x in range(AVERAGE_TYPES.TOTAL_AVERAGE_TYPES)] for y in range(AVERAGES.TOTAL_AVERAGES)]
    with open(results_path, "r") as f:
        total_data = f.readlines()
    average_counter = 0
    average_types_counter = 0
    for i in range(RESULTS.TOTAL_RESULTS + (AVERAGES.TOTAL_AVERAGES * AVERAGE_TYPES.TOTAL_AVERAGE_TYPES)):
        if(i < RESULTS.TOTAL_RESULTS):
            results[i] = float(total_data[i])
        elif(average_types_counter < AVERAGE_TYPES.TOTAL_AVERAGE_TYPES):
            average_values[average_counter][average_types_counter] = float(total_data[i])
            average_types_counter += 1
        else:
            average_types_counter = 0
            average_counter += 1
            average_values[average_counter][average_types_counter] = float(total_data[i])
            average_types_counter += 1
    return results, average_values
    

def processResults(paramaterized_data):
    paramaterized_data.new_simulation()
    results, average_values = readData()
    for i in range(RESULTS.TOTAL_RESULTS):
        paramaterized_data.add_results(i, results[i])
        
    for x in range(AVERAGES.TOTAL_AVERAGES):
        if (x == AVERAGES.TIME_BETWEEN_SPAWNS):
            paramaterized_data.add_averages(x, AVERAGE_TYPES.AVERAGES, average_values[x][AVERAGE_TYPES.AVERAGES])
            continue
        for y in range(AVERAGE_TYPES.TOTAL_AVERAGE_TYPES):
            if (y == AVERAGE_TYPES.LEFT_AVERAGES):
                if (results[RESULTS.PERCENT_LEFT_VEHICLES] == 0.0):
                    continue
            elif (y == AVERAGE_TYPES.SDV_LEFT_AVERAGES):
                if(results[RESULTS.PERCENT_SDV_LEFT] == 0.0):
                    continue
            elif (y == AVERAGE_TYPES.HD_LEFT_AVERAGES):
                if(results[RESULTS.PERCENT_HDV_LEFT] == 0.0):
                    continue
            elif (y == AVERAGE_TYPES.STRAIGHT_AVERAGES):
                if (results[RESULTS.PERCENT_STRAIGHT_VEHICLES] == 0.0):
                    continue
            elif (y == AVERAGE_TYPES.SDV_STRAIGHT_AVERAGES):
                if(results[RESULTS.PERCENT_SDV_STRAIGHT] == 0.0):
                    continue
            elif (y == AVERAGE_TYPES.HD_STRAIGHT_AVERAGES):
                if(results[RESULTS.PERCENT_HDV_STRAIGHT] == 0.0):
                    continue
            elif (y == AVERAGE_TYPES.RIGHT_AVERAGES):
                if (results[RESULTS.PERCENT_RIGHT_VEHICLES] == 0.0):
                    continue
            elif (y == AVERAGE_TYPES.SDV_RIGHT_AVERAGES):
                if(results[RESULTS.PERCENT_SDV_RIGHT] == 0.0):
                    continue
            elif (y == AVERAGE_TYPES.HD_RIGHT_AVERAGES):
                if(results[RESULTS.PERCENT_HDV_RIGHT] == 0.0):
                    continue
            elif (y == AVERAGE_TYPES.SELF_DRIVING_AVERAGES):
                if(results[RESULTS.PERCENT_SDV] == 0.0):
                    continue
            elif (y == AVERAGE_TYPES.HUMAN_DRIVING_AVERAGES):
                if(results[RESULTS.PERCENT_HDV] == 0.0):
                    continue
            paramaterized_data.add_averages(x, y, average_values[x][y])

def printTestOuptut(parameter_data_, filepath_):
    with open(filepath_, "w") as f:
        for i in range(RESULTS.TOTAL_RESULTS):
            print(parameter_data_.simulation_results[i], file=f)
        for i in range(AVERAGES.TOTAL_AVERAGES):
            for j in range(AVERAGE_TYPES.TOTAL_AVERAGE_TYPES):
                print(parameter_data_.simulation_averages[i][j], file=f)

def graphTravelTimeWRTLoad(data_, image_number_):
    title = "Time Through Intersection vs. Load"
    x_at_i = [0 for x in range(len(data_))]
    y_at_i = [0 for y in range(len(data_))]
    for i in range(len(data_)):
        x_at_i[i] = data_[i].simulation_averages[AVERAGES.TIME_BETWEEN_SPAWNS][AVERAGE_TYPES.AVERAGES]
        y_at_i[i] = data_[i].simulation_averages[AVERAGES.TIME_THROUGH_INTERSECTION][AVERAGE_TYPES.AVERAGES]
        plt.plot(x_at_i[i], y_at_i[i], marker='o', color='blue')
    plt.title(title, loc='right')
    plt.xlabel("Load [s]")
    plt.ylabel("Time [s]")
    plt.savefig(os.path.join(os.getcwd(),images_output_path_str+title+str(image_number_)+".png"),format="png", dpi=500)
    plt.close()
    a1, b1 = np.polyfit(x_at_i, y_at_i, 1)
    x_range = np.linspace(min(x_at_i), max(x_at_i), 50)
    plt.plot(x_range, a1 * x_range + b1, color="red")
    plt.title(title, loc='right')
    plt.xlabel("Load [s]")
    plt.ylabel("Time [s]")
    plt.savefig(os.path.join(os.getcwd(),images_output_path_str+title+LINE_STR+str(image_number_)+".png"),format="png", dpi=500)
    plt.close()

def graphTravelTimeWRTLoadPerVehicleType(data_, image_number_):
    title = "Time Through Intersection vs. Load"
    x_at_i = [0 for x in range(len(data_))]
    y1_at_i = [0 for y in range(len(data_))]
    y2_at_i = [0 for y in range(len(data_))]
    for i in range(len(data_)):
        x_at_i[i] = data_[i].simulation_averages[AVERAGES.TIME_BETWEEN_SPAWNS][AVERAGE_TYPES.AVERAGES]
        y1_at_i[i] = data_[i].simulation_averages[AVERAGES.TIME_THROUGH_INTERSECTION][AVERAGE_TYPES.HUMAN_DRIVING_AVERAGES]
        plt.plot(x_at_i[i], y1_at_i[i], marker='o', color="red", label="Human-Driving")
        if(data_[i].simulation_results[RESULTS.PERCENT_SDV] != 0):
            y2_at_i[i] = data_[i].simulation_averages[AVERAGES.TIME_THROUGH_INTERSECTION][AVERAGE_TYPES.SELF_DRIVING_AVERAGES]
            plt.plot(x_at_i[i], y2_at_i[i], marker='o', color="blue", label="Self-Driving")
    plt.title(title, loc='right')
    plt.xlabel("Load [s]")
    plt.ylabel("Time [s]")
    plt.savefig(os.path.join(os.getcwd(),images_output_path_str+title+"Per Vehicle Type"+str(image_number_)+".png"),format="png", dpi=500)
    plt.close()
    a1, b1 = np.polyfit(x_at_i, y1_at_i, 1)
    a2, b2 = np.polyfit(x_at_i, y2_at_i, 1)
    x_range = np.linspace(min(x_at_i), max(x_at_i), 50)
    plt.plot(x_range, a1 * x_range + b1, color="red", label="Human-Driving")
    plt.plot(x_range, a2 * x_range + b2, color="blue", label="Self-Driving")
    plt.title(title, loc='right')
    plt.xlabel("Load [s]")
    plt.ylabel("Time [s]")
    plt.legend()
    plt.savefig(os.path.join(os.getcwd(),images_output_path_str+title+"Per Vehicle Type"+LINE_STR+str(image_number_)+".png"),format="png", dpi=500)
    plt.close()

def graphCollisionPercentPerLoad(data_, image_number_):
    title = "Collision Percent vs. Load"
    percentages = [0 for x in range(len(data_))]
    x_at_i = [0 for x in range(len(data_))]
    for i in range(len(data_)):
        percentages[i] = data_[i].number_of_collisions / (data_[i].number_of_simulations - data_[i].number_of_fails)
        x_at_i[i] = data_[i].simulation_averages[AVERAGES.TIME_BETWEEN_SPAWNS][AVERAGE_TYPES.AVERAGES]
        plt.plot(x_at_i[i], percentages[i], marker='o', color="blue")
    plt.title(title, loc='right')
    plt.xlabel("Load [s]")
    plt.ylabel("Percent Collisions")
    plt.savefig(os.path.join(os.getcwd(),images_output_path_str+title+str(image_number_)+".png"),format="png", dpi=500)
    plt.close()
    a1, b1 = np.polyfit(x_at_i, percentages, 1)
    x_range = np.linspace(min(x_at_i), max(x_at_i), 50)
    plt.plot(x_range, a1 * x_range + b1, color="blue")
    plt.title(title, loc='right')
    plt.xlabel("Load [s]")
    plt.ylabel("Percent Collisions")
    plt.savefig(os.path.join(os.getcwd(),images_output_path_str+title+LINE_STR+str(image_number_)+".png"),format="png", dpi=500)
    plt.close()

def graphTravelTimeWRTSDVPercent(data_, image_number_):
    title = "Travel Time vs. Percent SDVs"
    x_at_i = [0 for x in range(len(data_))]
    y_at_i = [0 for y in range(len(data_))]
    for i in range(len(data_)):
        x_at_i[i] = data_[i].simulation_results[RESULTS.PERCENT_SDV]
        y_at_i[i] = data_[i].simulation_averages[AVERAGES.TIME_THROUGH_INTERSECTION][AVERAGE_TYPES.AVERAGES]
        plt.plot(x_at_i[i], y_at_i[i], marker='o', color="blue")
    plt.title(title, loc='right')
    plt.xlabel("Percent Self-Driving Vehicles")
    plt.ylabel("Time [s]")
    plt.savefig(os.path.join(os.getcwd(),images_output_path_str+title+str(image_number_)+".png"),format="png", dpi=500)
    plt.close()
    a1, b1 = np.polyfit(x_at_i, y_at_i, 1)
    x_range = np.linspace(min(x_at_i), max(x_at_i), 50)
    plt.plot(x_range, a1 * x_range + b1, color="blue")
    plt.title(title, loc='right')
    plt.xlabel("Percent Self-Driving Vehicles")
    plt.ylabel("Completion Time [s]")
    plt.savefig(os.path.join(os.getcwd(),images_output_path_str+title+LINE_STR+str(image_number_)+".png"),format="png", dpi=500)
    plt.close()

def graphCompletionTimeWRTLoad(data_, image_number_):
    title = "Completion Time vs. Load"
    x_at_i = [0 for x in range(len(data_))]
    y_at_i = [0 for y in range(len(data_))]
    for i in range(len(data_)):
        x_at_i[i] = data_[i].simulation_averages[AVERAGES.TIME_BETWEEN_SPAWNS][AVERAGE_TYPES.AVERAGES]
        y_at_i[i] = data_[i].simulation_results[RESULTS.ELAPSED_TIME]
        plt.plot(x_at_i[i], y_at_i[i], marker='o', color="blue")
    plt.title(title, loc='right')
    plt.xlabel("Load [s]")
    plt.ylabel("Completion Time [s]")
    plt.savefig(os.path.join(os.getcwd(),images_output_path_str+title+str(image_number_)+".png"),format="png", dpi=500)
    plt.close()
    a1, b1 = np.polyfit(x_at_i, y_at_i, 1)
    x_range = np.linspace(min(x_at_i), max(x_at_i), 50)
    plt.plot(x_range, a1 * x_range + b1, color="blue")
    plt.title(title, loc='right')
    plt.xlabel("Load [s]")
    plt.ylabel("Completion Time [s]")
    plt.savefig(os.path.join(os.getcwd(),images_output_path_str+title+LINE_STR+str(image_number_)+".png"),format="png", dpi=500)
    plt.close()

def graphEmissionsWRTLoad(data_, image_number_):
    title = "Emissions vs. Load"
    x_at_i = [0 for x in range(len(data_))]
    y_at_i = [0 for y in range(len(data_))]
    for i in range(len(data_)):
        x_at_i[i] = data_[i].simulation_averages[AVERAGES.TIME_BETWEEN_SPAWNS][AVERAGE_TYPES.AVERAGES]
        y_at_i[i] = data_[i].simulation_averages[AVERAGES.CO2_EMISSIONS][AVERAGE_TYPES.AVERAGES]
        plt.plot(x_at_i[i], y_at_i[i], marker='o', color="blue")
    plt.title(title, loc='right')
    plt.xlabel("Load [s]")
    plt.ylabel("CO2 Emissions [kg]")
    plt.savefig(os.path.join(os.getcwd(),images_output_path_str+title+str(image_number_)+".png"),format="png", dpi=500)
    plt.close()
    a1, b1 = np.polyfit(x_at_i, y_at_i, 1)
    x_range = np.linspace(min(x_at_i), max(x_at_i), 50)
    plt.plot(x_range, a1 * x_range + b1, color="blue")
    plt.title(title, loc='right')
    plt.xlabel("Load [s]")
    plt.ylabel("CO2 Emissions [kg]")
    plt.savefig(os.path.join(os.getcwd(),images_output_path_str+title+LINE_STR+str(image_number_)+".png"),format="png", dpi=500)
    plt.close()

def graphEmissionsWRTLoadPerVehicleType(data_, image_number_):
    title = "Emissions vs. Load"
    x_at_i = [0 for x in range(len(data_))]
    y1_at_i = [0 for y in range(len(data_))]
    y2_at_i = [0 for y in range(len(data_))]
    for i in range(len(data_)):
        x_at_i[i] = data_[i].simulation_averages[AVERAGES.TIME_BETWEEN_SPAWNS][AVERAGE_TYPES.AVERAGES]
        y1_at_i[i] = data_[i].simulation_averages[AVERAGES.CO2_EMISSIONS][AVERAGE_TYPES.HUMAN_DRIVING_AVERAGES]
        plt.plot(x_at_i[i], y1_at_i[i], marker='o', color="red", label="Human-Driving")
        if(data_[i].simulation_results[RESULTS.PERCENT_SDV] != 0):
            y2_at_i[i] = data_[i].simulation_averages[AVERAGES.CO2_EMISSIONS][AVERAGE_TYPES.SELF_DRIVING_AVERAGES]
            plt.plot(x_at_i[i], y2_at_i[i], marker='o', color="blue", label="Self-Driving")
    plt.title(title, loc='right')
    plt.xlabel("Load [s]")
    plt.ylabel("CO2 Emissions [kg]")
    plt.savefig(os.path.join(os.getcwd(),images_output_path_str+title+"Per Vehicle Type"+str(image_number_)+".png"),format="png", dpi=500)
    plt.close()
    a1, b1 = np.polyfit(x_at_i, y1_at_i, 1)
    a2, b2 = np.polyfit(x_at_i, y2_at_i, 1)
    x_range = np.linspace(min(x_at_i), max(x_at_i), 50)
    plt.plot(x_range, a1 * x_range + b1, color="red", label="Human-Driving")
    plt.plot(x_range, a2 * x_range + b2, color="blue", label="Self-Driving")
    plt.title(title, loc='right')
    plt.xlabel("Load [s]")
    plt.ylabel("CO2 Emissions [kg]")
    plt.legend()
    plt.savefig(os.path.join(os.getcwd(),images_output_path_str+title+"Per Vehicle Type"+LINE_STR+str(image_number_)+".png"),format="png", dpi=500)
    plt.close()
    

def scripting():
    start_time = time.time()
    number_of_collisions = 0
    number_of_fails = 0
    number_of_runs = 10
    run_number = 0
    total_runs = 0
    spawn_density_range = 101 #starts at 1 ends at value
    self_driving_vehicle_probability_range = 100 #starts at 0 ends at value-1
    spawn_density_increment = 20
    self_driving_vehicle_probability_increment = 25

    if(not os.path.exists(images_output_path)):
        os.mkdir(images_output_path)
    #makes sure the output tree is clean before beginning
    if(collision_text_path.is_file()):
        os.remove(collision_text_path)
    if(all_collisions_text_path.is_file()):
        os.remove(all_collisions_text_path)
    if(fail_report_text_path.is_file()):
        os.remove(fail_report_text_path)
    if(all_fail_report_text_path.is_file()):
        os.remove(all_fail_report_text_path)
    if(results_path.is_file()):
        os.remove(results_path)
    if(test_output_path.is_file()):
        os.remove(test_output_path)
    if(second_output_path.is_file()):
        os.remove(second_output_path)
    if(completed_swerrs.is_file()):
        os.remove(completed_swerrs)

    data_list = []
    percent_completion_str = "%% complete"

    
    quantity_of_runs = 0
    for x in range(1, spawn_density_range + 1, spawn_density_increment):
        for y in range(0, self_driving_vehicle_probability_range + 1, self_driving_vehicle_probability_increment):
            for z in range(number_of_runs):
                quantity_of_runs += 1

    print(quantity_of_runs)
    
    for x in range(1, spawn_density_range + 1, spawn_density_increment):
        for y in range(0, self_driving_vehicle_probability_range + 1, self_driving_vehicle_probability_increment):
            print(str(x) + " " + str(y))
            print(str((total_runs / quantity_of_runs) * 100) + percent_completion_str)
            with open(simulation_params_input, "w") as params:
                print(x, file=params)
                print(y, file=params, end="")
            param_data = data(x, y)
            while(run_number < number_of_runs):
                subprocess.run("source.exe")
                
                total_runs += 1
                run_number += 1

                if collision_text_path.is_file():
                    print("Collision Occured")
                    collision_data = []
                    swerr_data = []
                    with open(SWERR_path, "r") as swerrs:
                        swerr_data = swerrs.readlines()
                    with open(collision_text_path, "r") as input:
                        collision_data = input.readlines()
                    with open(all_collisions_text_path, "a") as output:
                        for line in collision_data:
                            print(line, file=output, end='')
                        print(file=output)
                        for swerr in swerr_data:
                            print(swerr, file=output, end='')
                        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", file=output)
                    os.remove(collision_text_path)
                    number_of_collisions += 1
                    param_data.add_collision()

                if fail_report_text_path.is_file():
                    print("Failure Occured")
                    failure_data = []
                    fail_swerr_data = []
                    with open(SWERR_path, "r") as swerrs:
                        fail_swerr_data = swerrs.readlines()
                    with open(fail_report_text_path, "r") as input:
                        failure_data = input.readlines()
                    with open(all_fail_report_text_path, "a") as output:
                        for line in failure_data:
                            print(line, file=output, end='')
                        print(file=output)
                        for swerr in fail_swerr_data:
                            print(swerr, file=output, end='')
                        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", file=output)
                    os.remove(fail_report_text_path)
                    number_of_fails += 1
                    param_data.add_failure()
                
                if results_path.is_file():
                    completion_swerr_data = []
                    with open(SWERR_path, "r") as swerrs:
                        completion_swerr_data = swerrs.readlines()
                    with open(completed_swerrs, "a") as output:
                        for swerr in completion_swerr_data:
                            print(swerr, file=output, end='')
                        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", file=output)
                    processResults(param_data)
                    os.remove(results_path)

            print(param_data.number_of_collisions)
            print(param_data.number_of_fails)
            print(run_number)
            param_data.finalize()
            data_list.append(param_data)
            run_number = 0
            print()
    
    print(number_of_collisions)
    print(number_of_fails)
    print(total_runs)
    print(time.time()-start_time)
    graphTravelTimeWRTLoad(data_list, 1)
    graphTravelTimeWRTLoadPerVehicleType(data_list, 1)
    graphCollisionPercentPerLoad(data_list, 1)
    graphTravelTimeWRTSDVPercent(data_list, 1)
    graphCompletionTimeWRTLoad(data_list, 1)
    graphEmissionsWRTLoad(data_list, 1)
    graphEmissionsWRTLoadPerVehicleType(data_list, 1)

def basicScripting():
    start_time = time.time()
    number_of_collisions = 0
    number_of_fails = 0
    number_of_runs = 100
    run_number = 0
    spawn_density = STANDARD_SIMULATION_PARAMS[0]
    self_driving_vehicle_probability = STANDARD_SIMULATION_PARAMS[1]

    if(collision_text_path.is_file()):
        os.remove(collision_text_path)
    if(all_collisions_text_path.is_file()):
        os.remove(all_collisions_text_path)
    if(fail_report_text_path.is_file()):
        os.remove(fail_report_text_path)
    if(all_fail_report_text_path.is_file()):
        os.remove(all_fail_report_text_path)
    if(results_path.is_file()):
        os.remove(results_path)
    if(test_output_path.is_file()):
        os.remove(test_output_path)
    if(second_output_path.is_file()):
        os.remove(second_output_path)
    if(completed_swerrs.is_file()):
        os.remove(completed_swerrs)


    param_data = data(spawn_density, self_driving_vehicle_probability)
    with open(simulation_params_input, "w") as params:
                print(spawn_density, file=params)
                print(self_driving_vehicle_probability, file=params, end="")
    while(run_number < number_of_runs):
        subprocess.run("source.exe")
        
        run_number += 1

        if collision_text_path.is_file():
            print("Collision Occured")
            collision_data = []
            swerr_data = []
            with open(SWERR_path, "r") as swerrs:
                swerr_data = swerrs.readlines()
            with open(collision_text_path, "r") as input:
                collision_data = input.readlines()
            with open(all_collisions_text_path, "a") as output:
                for line in collision_data:
                    print(line, file=output, end='')
                print(file=output)
                for swerr in swerr_data:
                    print(swerr, file=output, end='')
                print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", file=output)
            os.remove(collision_text_path)
            number_of_collisions += 1
            param_data.add_collision()

        if fail_report_text_path.is_file():
            print("Failure Occured")
            failure_data = []
            fail_swerr_data = []
            with open(SWERR_path, "r") as swerrs:
                fail_swerr_data = swerrs.readlines()
            with open(fail_report_text_path, "r") as input:
                failure_data = input.readlines()
            with open(all_fail_report_text_path, "a") as output:
                for line in failure_data:
                    print(line, file=output, end='')
                print(file=output)
                for swerr in fail_swerr_data:
                    print(swerr, file=output, end='')
                print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", file=output)
            os.remove(fail_report_text_path)
            number_of_fails += 1
            param_data.add_failure()
        
        if results_path.is_file():
            completion_swerr_data = []
            with open(SWERR_path, "r") as swerrs:
                completion_swerr_data = swerrs.readlines()
            with open(completed_swerrs, "a") as output:
                for swerr in completion_swerr_data:
                    print(swerr, file=output, end='')
                print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~", file=output)
            processResults(param_data)
            os.remove(results_path)

    print(number_of_collisions)
    print(number_of_fails)
    print(run_number)
    #printTestOuptut(param_data, second_output_path)
    param_data.finalize()
    printTestOuptut(param_data, test_output_path)
    print(time.time()-start_time)
    
#basicScripting()
scripting()