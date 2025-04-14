from simulation import *
from config import *
import numpy as np

# RFID positions
rfid_positions = [
    (10.1, 5.5),
    (25.1, 7.3),
    (40.3, 3.3),
    (35.6, 9.1),
    (15.6, 1)
]

robot_config = RobotConfig(circle_radius=1, front_distance=3, angle_range=np.pi/4)
room_config = RoomConfig(room_width=50, room_height=20, horizontal_step=1, vertical_step=1)

if __name__ == "__main__":
    rfid_rmse, rfid_areas = startSimulation(robot_config, room_config, rfid_positions, True)

    rmse_values = [e for e in rfid_rmse.values() if e is not None]
    if rmse_values:
        overall_rmse = np.sqrt(np.mean(np.square(rmse_values)))
    else:
        overall_rmse = None

    print("RFID Areas:")
    for i in rfid_areas:
        if rfid_areas[i] is not None:
            print(f"RFID {i}: Area = {rfid_areas[i]:.2f}")
        else:
            print(f"RFID {i}: Area = Not Detected")

    print("\nRFID RMSE Errors:")
    for i in rfid_rmse:
        if rfid_rmse[i] is not None:
            print(f"RFID {i}: RMSE = {rfid_rmse[i]:.2f}")
        else:
            print(f"RFID {i}: RMSE = Not Detected")

    if overall_rmse is not None:
        print(f"\nOverall RMSE: {overall_rmse:.2f}")
    else:
        print("\nOverall RMSE: Not Calculated (No RFIDs detected)")