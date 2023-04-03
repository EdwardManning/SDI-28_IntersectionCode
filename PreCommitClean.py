import os
from pathlib import Path
import shutil

STANDARD_SIMULATION_PARAMS = [1, 25, 1]

collision_text_path = Path('./Output/CollisionInformation.txt')
all_collisions_text_path = Path('./Output/AllCollisions.txt')
fail_report_text_path = Path('./Output/FailureReport.txt')
all_fail_report_text_path = Path('./Output/AllFailures.txt')
results_path = Path('./Output/PythonResults.txt')
test_output_path = Path('./Output/PythonTestOutput.txt')
second_output_path = Path('./Output/AuxPythonTestOutput.txt')
simulation_params_input = Path('./Input/SimulationParamsInput.txt')
debug_log_path = Path('./Output/DebugLog.txt')
events_path = Path('./Output/Events.txt')
formatted_results_path = Path('./Output/Results.txt')
SWERR_path = Path('./Output/SwerrList.txt')
vehicle_outputs_path = Path('./Output/VehicleOutput')
completed_swerrs = Path('./Output/CompletionSwerrs.txt')
images_output_path_str = "./Output/Images/"
images_output_path = Path(images_output_path_str)

DEFAULT_SWERR_STR = "SWERR Occurances:"


def cleanRepo():
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
    if(debug_log_path.is_file()):
        os.remove(debug_log_path)
    if(completed_swerrs.is_file()):
        os.remove(completed_swerrs)
    if(events_path.is_file()):
        with open(events_path, "w") as f:
            print(end='', file=f)
    if(formatted_results_path.is_file()):
        with open(formatted_results_path, "w") as f:
            print(end='', file=f)
    if(SWERR_path.is_file()):
        with open(SWERR_path, "w") as f:
            print(DEFAULT_SWERR_STR, file=f)
    if(os.listdir(vehicle_outputs_path)):
        shutil.rmtree(vehicle_outputs_path)
        os.mkdir(vehicle_outputs_path)
    if(simulation_params_input.is_file()):
        with open(simulation_params_input, "w") as f:
            print(STANDARD_SIMULATION_PARAMS[0], file=f)
            print(STANDARD_SIMULATION_PARAMS[1], file=f)
            print(STANDARD_SIMULATION_PARAMS[2], end='', file=f)
    if(os.path.isdir(images_output_path)):
        remove_path = input("Image File Present, would you like to delete it? (make sure all relevent data is saved first) (y/n) ")
        if(remove_path == 'y'):
            shutil.rmtree(images_output_path)
        else:
            print(images_output_path_str + " was not removed.")
            
cleanRepo()

    
    