import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from path import *

def startSimulation(robot_config, room_config, rfid_positions):
    # RFID index to detection points
    rfid_detections = {i: [] for i in range(len(rfid_positions))}

    robot_path = generate_robot_path(room_config.room_width, room_config.room_height, room_config.horizontal_step, room_config.vertical_step)
    # Simulation
    for x, y, orientation in robot_path:
        position = (x, y)
        detection_shape = get_robot_detection_shape(position, orientation, robot_config)

        # Check which RFIDs are within the detection shape
        for i, rfid_pos in enumerate(rfid_positions):
            rfid_point = Point(rfid_pos)
            if detection_shape.contains(rfid_point):
                rfid_detections[i].append(position)

    # Estimating zones and calculating area
    rfid_zones = {}
    rfid_areas = {}
    rfid_rmse = {}

    for i, detections in rfid_detections.items():
        # For each detection point, create the robot's detection shape and collect them
        shapes = []
        for det_pos in detections:
            for x_p, y_p, orientation_p in robot_path:
                if (x_p, y_p) == det_pos:
                    shape = get_robot_detection_shape(det_pos, orientation_p, robot_config)
                    shapes.append(shape)
                    break
        if shapes:
            # Intersect all shapes to estimate the RFID zone
            estimated_zone = shapes[0]
            for shape in shapes[1:]:
                estimated_zone = estimated_zone.intersection(shape)
            rfid_zones[i] = estimated_zone

            rfid_areas[i] = estimated_zone.area

            # Calculate RMSE error (distance between estimated center and actual position)
            estimated_centroid = estimated_zone.centroid
            actual_position = Point(rfid_positions[i])
            error = estimated_centroid.distance(actual_position)
            rfid_rmse[i] = error
        else:
            # RFID was never detected
            rfid_zones[i] = None
            rfid_areas[i] = None
            rfid_rmse[i] = None

    # RMSE
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

    # Visualization
    fig, ax = plt.subplots(figsize=(12, 6))

    # Plot the room
    ax.set_xlim(0, room_config.room_width)
    ax.set_ylim(0, room_config.room_height)
    ax.set_aspect('equal')

    # Plot RFIDs
    for rfid_pos in rfid_positions:
        ax.plot(rfid_pos[0], rfid_pos[1], 'ro', label='RFID')

    # Plot robot path
    path_x = [pos[0] for pos in robot_path]
    path_y = [pos[1] for pos in robot_path]
    ax.plot(path_x, path_y, 'k--', linewidth=0.5, label='Robot Path')

    robot_marker, = ax.plot([], [], 'bo', label='Robot')
    detection_patch = None

    def init():
        robot_marker.set_data([], [])
        return robot_marker,

    def update(frame):
        global detection_patch
        x, y, orientation = robot_path[frame]
        position = (x, y)
        robot_marker.set_data(position[0], position[1])

        # Remove previous detection shape
        # if detection_patch is not None:
        #     detection_patch.remove()

        # Plot detection shape
        detection_shape = get_robot_detection_shape(position, orientation, robot_config)
        if not detection_shape.is_empty and isinstance(detection_shape, Polygon):
            x_detection, y_detection = detection_shape.exterior.xy
            detection_patch = ax.fill(x_detection, y_detection, alpha=0.3, fc='blue')[0]
        else:
            detection_patch = None

        return robot_marker,

    ani = FuncAnimation(fig, update, frames=len(robot_path), init_func=init,
                        blit=False, interval=100, repeat=False)

    for i, zone in rfid_zones.items():
        if zone and not zone.is_empty:
            x_zone, y_zone = zone.exterior.xy
            ax.fill(x_zone, y_zone, alpha=0.5, label=f'RFID {i} Estimated Zone')

    ax.legend()
    plt.title("RFID Localization Simulation")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.show()