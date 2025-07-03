"""
Generate random instances.
Solve the instances and plot the solutions.
"""

import random
import subprocess
import mdhur as solver

random.seed(100)

def generate_random_instance(
    num_vehicles,
    num_targets,
    speed_range,
    xmin, xmax,
    ymin, ymax
):
    """
    Randomly generate vehicle and target positions.
    
    Parameters:
        num_vehicles (int): Number of vehicles to generate.
        num_targets (int): Number of targets to generate.
        speed_range (tuple): (min_speed, max_speed) for vehicle speeds.
        xmin, xmax (float): Bounds for x coordinates.
        ymin, ymax (float): Bounds for y coordinates.
        
    Returns:
        Veh (list of [x, y, speed]): Vehicle positions and speeds.
        Tar (list of [x, y]): Target positions.
    """
    Veh = [
        [
            round(random.uniform(xmin, xmax), 2),
            round(random.uniform(ymin, ymax), 2),
            round(random.uniform(*speed_range), 2)
        ]
        for _ in range(num_vehicles)
    ]
    
    Tar = [
        [
            round(random.uniform(xmin, xmax), 2),
            round(random.uniform(ymin, ymax), 2)
        ]
        for _ in range(num_targets)
    ]
    
    return Veh, Tar



if __name__ == "__main__":

    # DEFINE INSTANCE PARAMETERS
    num_vehicles = 10
    num_targets = 100
    vmin, vmax = 1, 1
    xmin, xmax, ymin, ymax = -50, 50, -50, 50

    # USER SHOULD MODIFY THESE TWO LINES BASED ON THEIR LKH PATH AND DESIRED LKH FILE NAMES
    file_name = "test_matrix"
    LKH_path = "../../LKH/LKH-2.0.9/"

    # GENERATE AND SOLVE THE INSTANCE
    veh, tar = generate_random_instance(num_vehicles, num_targets, (vmin, vmax), xmin, xmax, ymin, ymax)
    transform = solver.TSP(len(veh), len(tar), veh, tar)

    LKH_inst = solver.transformation_algorithm(transform)
    file_path = LKH_path + file_name

    LKH_1 = solver.LKH_file_generator(LKH_inst, file_path + '.tsp', file_path + '.par', file_path + '_sol')
    LKH_1.create_cost_matrix_TSP()
    LKH_1.create_cost_matrix_PAR(file_path + '.tsp', file_path + '_sol')
    cmd = [LKH_path + 'LKH', file_path + '.par']
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)

    for line in process.stdout: # Print output as it is produced
        print(line, end="")  # 'end=""' avoids double newlines
    process.wait() 

    # READ LKH SOLUTION, TRANSFORM, AND PLOT
    LKH_sol = LKH_1.read_sol()
    # print(LKH_sol)
    solver.plot_transformed_LKH(transform, LKH_inst, LKH_sol)

