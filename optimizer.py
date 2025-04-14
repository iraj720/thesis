import numpy as np
from simulation import *
from config import RobotConfig, RoomConfig
import itertools
import time

# RFID positions
rfid_positions = [
    (10.1, 5.5),
    (25.1, 7.3),
    (40.3, 3.3),
    (35.6, 9.1),
    (15.6, 1)
]

def main():
    # Define parameter ranges
    circle_radius_range = np.arange(0.5, 2.5, 0.5)        # Example: 0.5 to 2.0 with step 0.5
    front_distance_range = np.arange(2, 6, 1)            # Example: 2 to 5 with step 1
    angle_range = [np.pi/6, np.pi/4, np.pi/3]            # Example: 30°, 45°, 60°
    room_width_range = [50, 60]                           # Example: 50 and 60 units
    room_height_range = [10, 15]                          # Example: 10 and 15 units
    horizontal_step_range = [1, 2]                        # Example: step sizes 1 and 2
    vertical_step_range = [1, 2]                          # Example: step sizes 1 and 2
    
    # Create all possible combinations
    parameter_combinations = list(itertools.product(
        circle_radius_range,
        front_distance_range,
        angle_range,
        room_width_range,
        room_height_range,
        horizontal_step_range,
        vertical_step_range
    ))
    
    print(f"Total parameter combinations to evaluate: {len(parameter_combinations)}")
    
    best_rmse = float('inf')
    best_parameters = None
    best_results = None
    
    # To track progress
    start_time = time.time()
    
    for idx, (circle_radius, front_distance, angle_range_val, room_width, room_height, horizontal_step, vertical_step) in enumerate(parameter_combinations, 1):
        print(f"\nRunning simulation {idx}/{len(parameter_combinations)} with parameters:")
        print(f"  Circle Radius: {circle_radius}")
        print(f"  Front Distance: {front_distance}")
        print(f"  Angle Range: {np.degrees(angle_range_val):.1f} degrees")
        print(f"  Room Width: {room_width}")
        print(f"  Room Height: {room_height}")
        print(f"  Horizontal Step: {horizontal_step}")
        print(f"  Vertical Step: {vertical_step}")
        
        # Initialize configurations
        robot_config = RobotConfig(
            circle_radius=circle_radius,
            front_distance=front_distance,
            angle_range=angle_range_val
        )
        room_config = RoomConfig(
            room_width=room_width,
            room_height=room_height,
            horizontal_step=horizontal_step,
            vertical_step=vertical_step
        )
        
        # Run simulation
        rfid_rmse, rfid_areas = startSimulation(robot_config, room_config, rfid_positions, False)

        overall_rmse = 0.0
        rmse_values = [e for e in rfid_rmse.values() if e is not None]
        if rmse_values:
            overall_rmse = np.sqrt(np.mean(np.square(rmse_values)))
        else:
            overall_rmse = None
        
        # Check if this is the best RMSE so far
        if overall_rmse is not None and overall_rmse < best_rmse:
            best_rmse = overall_rmse
            best_parameters = {
                'circle_radius': circle_radius,
                'front_distance': front_distance,
                'angle_range_deg': np.degrees(angle_range_val),
                'room_width': room_width,
                'room_height': room_height,
                'horizontal_step': horizontal_step,
                'vertical_step': vertical_step
            }
            best_results = {
                'rfid_rmse': rfid_rmse,
                'rfid_areas': rfid_areas,
                'overall_rmse': overall_rmse
            }


    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"\nOptimization completed in {elapsed_time:.2f} seconds.")
    
    if best_parameters:
        print("\nBest Parameter Set Found:")
        for param, value in best_parameters.items():
            print(f"  {param}: {value}")
        print(f"  Overall RMSE: {best_rmse:.2f}")
    else:
        print("No valid simulations were run to compute RMSE.")

if __name__ == "__main__":
    main()