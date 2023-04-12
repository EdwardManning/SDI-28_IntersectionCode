import os
from pathlib import Path

output_file = Path('./CodeFile.txt')
intersection_simpy = Path('./intersection_sim.py')
precommitcleanpy = Path('./PreCommitClean.py')
sourcecpp = Path('./Source.cpp')
simulationh = Path('./Simulation.h')
simulationcpp = Path('./Simulation.cpp')
common_path = Path('./Common')
infrastructure_path = Path('./Infrastructure')
light_events_path = Path('./Infrastructure/LightEvents')
vehicle_path = Path('./Vehicles')



def print_code():
    if output_file.is_file():
        os.remove(output_file)
    code = []

    #intersection_sim.py
    with open(intersection_simpy, "r") as file:
        code = file.readlines()
    with open(output_file, "a") as code_out:
        print("$@$", file=code_out, end='')
        print(intersection_simpy, file=code_out)
        print(file=code_out)
        for line in code:
            print(line, file=code_out, end='')
        print(file=code_out)
        print(file=code_out)

    #PreCommitClean.py
    with open(precommitcleanpy, "r") as file:
        code = file.readlines()
    with open(output_file, "a") as code_out:
        print("$@$", file=code_out, end='')
        print(precommitcleanpy, file=code_out)
        print(file=code_out)
        for line in code:
            print(line, file=code_out, end='')
        print(file=code_out)
        print(file=code_out)

    #Source.cpp
    with open(sourcecpp, "r") as file:
        code = file.readlines()
    with open(output_file, "a") as code_out:
        print("$@$", file=code_out, end='')
        print(sourcecpp, file=code_out)
        print(file=code_out)
        for line in code:
            print(line, file=code_out, end='')
        print(file=code_out)
        print(file=code_out)

    #Simulation.cpp
    with open(simulationcpp, "r") as file:
        code = file.readlines()
    with open(output_file, "a") as code_out:
        print("$@$", file=code_out, end='')
        print(simulationcpp, file=code_out)
        print(file=code_out)
        for line in code:
            print(line, file=code_out, end='')
        print(file=code_out)
        print(file=code_out)

    #Simulation.h
    with open(simulationh, "r") as file:
        code = file.readlines()
    with open(output_file, "a") as code_out:
        print("$@$", file=code_out, end='')
        print(simulationh, file=code_out)
        print(file=code_out)
        for line in code:
            print(line, file=code_out, end='')
        print(file=code_out)
        print(file=code_out)

    #./Common
    for filename in os.listdir(common_path):
        with open(os.path.join(common_path, filename), "r") as f:
            code = f.readlines()
        with open(output_file, "a") as out:
            print("$@$", file=out, end='')
            print(os.path.join(common_path, filename), file=out)
            print(file=out)
            for line in code:
                print(line, file=out, end='')
            print(file=out)
            print(file=out)

    #./Infrastructure
    for filename in os.listdir(infrastructure_path):
        if not Path(os.path.join(infrastructure_path, filename)).is_file():
            continue
        with open(os.path.join(infrastructure_path, filename), "r") as f:
            code = f.readlines()
        with open(output_file, "a") as out:
            print("$@$", file=out, end='')
            print(os.path.join(infrastructure_path, filename), file=out)
            print(file=out)
            for line in code:
                print(line, file=out, end='')
            print(file=out)
            print(file=out)
    
    #./Infrastructure/LightEvents
    for filename in os.listdir(light_events_path):
        with open(os.path.join(light_events_path, filename), "r") as f:
            code = f.readlines()
        with open(output_file, "a") as out:
            print("$@$", file=out, end='')
            print(os.path.join(light_events_path, filename), file=out)
            print(file=out)
            for line in code:
                print(line, file=out, end='')
            print(file=out)
            print(file=out)

    #./Vehicles
    for filename in os.listdir(vehicle_path):
        with open(os.path.join(vehicle_path, filename), "r") as f:
            code = f.readlines()
        with open(output_file, "a") as out:
            print("$@$", file=out, end='')
            print(os.path.join(vehicle_path, filename), file=out)
            print(file=out)
            for line in code:
                print(line, file=out, end='')
            print(file=out)
            print(file=out)
print_code()