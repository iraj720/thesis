import numpy as np
from simulation import *
import itertools
import time

# RFID positions
rfid_positions = [
    (0.1, 5.5),
    (9.1, 7.3),
    (6.1, 4.3),
    (5.6, 1)
]

def main():
    # Define parameter ranges
    room_width_range = [10, 20]                         # Example: 50 and 60 units
    room_height_range = [10, 20]                        # Example: 10 and 15 units
    horizontal_step_range = np.arange(0.1, 2.1, 0.1).round(2).tolist()  # 0.1 to 2.0 with step 0.1
    vertical_step_range = np.arange(0.1, 2.1, 0.1).round(2).tolist()
    
    # Create all possible combinations
    parameter_combinations = list(itertools.product(
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
    
    for idx, (room_width, room_height, horizontal_step, vertical_step) in enumerate(parameter_combinations, 1):
        print(f"\nRunning simulation {idx}/{len(parameter_combinations)} with parameters:")
        print(f"  Room Width: {room_width}")
        print(f"  Room Height: {room_height}")
        print(f"  Horizontal Step: {horizontal_step}")
        print(f"  Vertical Step: {vertical_step}")
        
        # Initialize configurations
        robot_config = RobotConfig()
        room_config = RoomConfig(
            room_width=room_width,
            room_height=room_height,
            horizontal_step=horizontal_step,
            vertical_step=vertical_step
        )
        
        # Run simulation
        rfid_rmse, rfid_areas, mesures = startSimulation(robot_config, room_config, rfid_positions, False)

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