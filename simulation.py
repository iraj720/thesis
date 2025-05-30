import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from shapely.geometry import Point
from animation import *
from antenna import *
from path import *
from algorithm import *
import time

##############################
# Configuration Classes
##############################

class RobotConfig:
    def __init__(self, angle_range=np.pi / 2, Pt=1, lam=0.33, G0=2.0, m=4):
        """
        Configuration for the robot and its signal reception.

        Parameters:
        - angle_range: Half-angle (in radians) of the field of view for RFID detection.
        - rssi_sigma: Standard deviation of Gaussian noise added to RSSI measurements.
        - _30db_range: Range at which RSSI drops by 30dB (optional).
        - Pt: Transmit power.
        - lam: Wavelength of signal.
        - G0: Antenna gain.
        - m: Path-loss exponent.
        """
        self.angle_range = angle_range
        self.Pt = Pt
        self.lam = lam
        self.G0 = G0
        self.m = m

class RoomConfig:
    def __init__(self, room_width=50.0, room_height=10.0, horizontal_step=1.0, vertical_step=2.0):
        """
        Configuration for the dimensions of the room and robot scanning step size.

        Parameters:
        - room_width: Width of the room.
        - room_height: Height of the room.
        - horizontal_step: Step size in the horizontal direction.
        - vertical_step: Step size in the vertical direction.
        """
        self.room_width = room_width
        self.room_height = room_height
        self.horizontal_step = horizontal_step
        self.vertical_step = vertical_step

##############################
# Main Simulation Function
##############################

def startSimulation(robot_config, room_config, rfid_positions, withAnimation):
    """
    Runs the main simulation for RFID detection using a moving robot.

    Process:
    1. Generate robot path through the room.
    2. At each step, measure RSSI from each RFID.
    3. Convert RSSI measurements to feasible regions (annular sectors).
    4. Intersect regions to estimate RFID location.
    5. Optionally animate the process.

    Returns:
    - rfid_rmse: Dictionary of RMSE between estimated and actual RFID positions.
    - rfid_areas: Area of each estimated feasible zone.
    - rfid_measurements: RSSI measurement logs.
    """

    # Step 1: Generate robot's movement path
    robot_path = generate_robot_path(
        room_config.room_width,
        room_config.room_height,
        room_config.horizontal_step,
        room_config.vertical_step
    )

    antenna = CustomAntenna()

    # Step 2: Initialize RSSI measurement storage
    rfid_measurements = {i: [] for i in range(len(rfid_positions))}

    # Step 3: Take RSSI measurements at each step for each RFID
    for (x, y, orientation) in robot_path:
        robot_pos = (x, y)
        for i, rfid_pos in enumerate(rfid_positions):
            rssi = antenna.measure_rssi(rfid_pos[0], rfid_pos[1], robot_pos[0], robot_pos[1], orientation)
            rfid_measurements[i].append((robot_pos, orientation, rssi))

    # Step 4: Convert RSSI measurements into polygonal detection bands and intersect them
    rfid_zones = {}
    rfid_areas = {}
    rfid_rmse = {}
    all_shapes = {}

    for i, rfid_pos in enumerate(rfid_positions):
        measurement_list = rfid_measurements[i]
        shapes = []

        for (pos, orientation, rssival) in measurement_list:
            # Convert each RSSI value into an annular sector shape
            poly = detection_shape(orientation, rssival, antenna)

            # giving the right offset to detection shape
            poly = translate(poly, xoff=pos[0], yoff=pos[1])

            # add the detected shape to detected shapes for this RFID
            shapes.append(poly)

        # Step 5: Compute intersection of all shapes
        if not shapes:
            rfid_zones[i] = None
            rfid_areas[i] = None
            rfid_rmse[i] = None
        else:
            estimated_zone = shapes[0]
            for s in shapes[1:]:
                # intersect shapes
                if s.is_valid and estimated_zone.is_valid:
                    intersection = estimated_zone.intersection(s)
                    if not intersection.is_empty:
                        estimated_zone = intersection

            rfid_zones[i] = estimated_zone
            rfid_areas[i] = estimated_zone.area if not estimated_zone.is_empty else 0

            # Step 6: Estimate position and compute RMSE
            if not estimated_zone.is_empty:
                estimated_centroid = estimated_zone.centroid
                actual_position = Point(rfid_pos)
                # print("Estimated position:", estimated_centroid, "Actual position:", actual_position)
                error = estimated_centroid.distance(actual_position)
                rfid_rmse[i] = error
            else:
                rfid_rmse[i] = None

            all_shapes[i] = shapes

    if withAnimation:
        animate(room_config, robot_config, robot_path, rfid_zones, rfid_positions, all_shapes)

    return rfid_rmse, rfid_areas, rfid_measurements
