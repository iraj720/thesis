import numpy as np
import itertools
import time
import matplotlib.pyplot as plt
from simulation import startSimulation, RobotConfig, RoomConfig

# RFID positions
rfid_positions = [
    (0.1, 5.5),
    (9.1, 7.3),
    (6.1, 4.3),
    (5.6, 1)
]

def main():
    # Define parameter ranges
    room_width_range = [10, 20]
    room_height_range = [10, 20]
    horizontal_step_range = np.arange(0.1, 1.1, 0.1).round(2).tolist()
    vertical_step_range = np.arange(0.1, 1.1, 0.1).round(2).tolist()
    
    # Prepare lists to collect results per parameter
    results = {
        'room_width': [],
        'room_height': [],
        'horizontal_step': [],
        'vertical_step': [],
        'rmse': []
    }
    
    # Iterate through all combinations
    start_time = time.time()
    combinations = list(itertools.product(
        room_width_range,
        room_height_range,
        horizontal_step_range,
        vertical_step_range
    ))
    
    for room_width, room_height, h_step, v_step in combinations:
        robot_config = RobotConfig()
        room_config = RoomConfig(
            room_width=room_width,
            room_height=room_height,
            horizontal_step=h_step,
            vertical_step=v_step
        )
        
        rfid_rmse, _, _ = startSimulation(robot_config, room_config, rfid_positions, False)
        rmse_vals = [e for e in rfid_rmse.values() if e is not None]
        if not rmse_vals:
            continue
        overall_rmse = np.sqrt(np.mean(np.square(rmse_vals)))
        
        # Store results
        results['room_width'].append(room_width)
        results['room_height'].append(room_height)
        results['horizontal_step'].append(h_step)
        results['vertical_step'].append(v_step)
        results['rmse'].append(overall_rmse)
    
    elapsed = time.time() - start_time
    print(f"Completed simulations in {elapsed:.2f} sec, plotted {len(results['rmse'])} points.")

    # Plot graphs for each parameter vs RMSE
    for param in ['room_width', 'room_height', 'horizontal_step', 'vertical_step']:
        plt.figure()
        plt.scatter(results[param], results['rmse'])
        plt.title(f"{param.replace('_', ' ').title()} vs RMSE")
        plt.xlabel(param.replace('_', ' ').title())
        plt.ylabel("Overall RMSE")
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    main()
