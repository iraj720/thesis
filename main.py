from simulation import *
from config import *
import numpy as np

##############################
# Configuration and Setup
##############################

# Define the fixed positions of RFID tags in the room
rfid_positions = [
    (15.1, 12.5),
    (25.1, 7.3),
    (40.3, 3.3),
    (35.6, 9.1),
    (15.6, 1)
]

# Configure robot sensing properties and room dimensions
robot_config = RobotConfig(
    rssi_sigma=0,             # No RSSI noise
    angle_range=np.pi / 4     # 45-degree antenna field of view
)

room_config = RoomConfig(
    room_width=50,            # Width of the room
    room_height=20,           # Height of the room
    horizontal_step=1,        # Step size for robot movement (x-axis)
    vertical_step=1           # Step size for robot movement (y-axis)
)

##############################
# Main Simulation Execution
##############################

if __name__ == "__main__":
    # Run the simulation and gather results
    rfid_rmse, rfid_areas, rfid_values = startSimulation(
        robot_config,
        room_config,
        rfid_positions,
        withAnimation=True
    )

    # Compute overall RMSE (Root Mean Square Error) for detected RFID tags
    rmse_values = [e for e in rfid_rmse.values() if e is not None]
    if rmse_values:
        overall_rmse = np.sqrt(np.mean(np.square(rmse_values)))
    else:
        overall_rmse = None

    ##############################
    # Print Results
    ##############################

    print("📐 RFID Estimated Areas:")
    for i, area in rfid_areas.items():
        if area is not None:
            print(f"  RFID {i}: Area = {area:.5f}")
        else:
            print(f"  RFID {i}: Area = ❌ Not Detected")

    print("\n📊 RFID RMSE Errors:")
    for i, err in rfid_rmse.items():
        if err is not None:
            print(f"  RFID {i}: RMSE = {err:.5f}")
        else:
            print(f"  RFID {i}: RMSE = ❌ Not Detected")

    if overall_rmse is not None:
        print(f"\n✅ Overall RMSE: {overall_rmse:.5f}")
    else:
        print("\n⚠️ Overall RMSE: Not Calculated (No RFIDs detected)")
