import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import cascaded_union
import math

# Define the environment
room_width = 50
room_height = 10
horizontal_step = 1
vertical_step = 2

# Robot configuration class
class RobotConfig:
    def __init__(self, circle_radius=1, front_distance=5, angle_range=np.pi/2):
        """
        circle_radius: Radius of the circle at the back of the robot.
        front_distance: Maximum distance of the front detection shape.
        angle_range: Angular range for the front detection shape (in radians).
        """
        self.circle_radius = circle_radius
        self.front_distance = front_distance
        self.angle_range = angle_range  # e.g., np.pi/2 for +/- 90 degrees

# Initialize robot configuration with default values
robot_config = RobotConfig(circle_radius=1, front_distance=3, angle_range=np.pi/4)

# Generate robot path with orientation
def generate_robot_path(room_width, room_height, horizontal_step, vertical_step):
    path = []
    x = 0
    y = 0
    direction = 1 
    orientation = 0  # Initial orientation (0 degrees facing right)
    while y <= room_height:
        # Move horizontally
        x_end = room_width if direction == 1 else 0
        while (direction == 1 and x <= x_end) or (direction == -1 and x >= x_end):
            path.append((x, y, orientation))
            x += horizontal_step * direction
        # Correct x if necessary (since we may have moved beyond boundary)
        x = max(0, min(x, room_width))
        # At the end of horizontal movement, rotate 90 degrees to face up
        if y + vertical_step > room_height:
            break  # Do not proceed further if we've reached the max height
        if direction == 1:
            # Moving right, rotate CCW to face up
            orientation = (orientation + 90) % 360
        else:
            # Moving left, rotate CW to face up
            orientation = (orientation - 90) % 360
        path.append((x, y, orientation))  # Include rotation at current position
        # Move up
        y += vertical_step
        path.append((x, y, orientation))  # Include movement up
        # Rotate 90 degrees to face opposite horizontal direction
        if direction == 1:
            # From up to left, rotate CCW 90 degrees
            orientation = (orientation + 90) % 360
        else:
            # From up to right, rotate CW 90 degrees
            orientation = (orientation - 90) % 360
        path.append((x, y, orientation))  # Include rotation at current position
        # Change direction
        direction *= -1
    return path

robot_path = generate_robot_path(room_width, room_height, horizontal_step, vertical_step)

# Define the RFIDs positions
rfid_positions = [
    (10, 5),
    (25, 7),
    (40, 3),
    (35, 9),
    (15, 1)
]

# Create a map from RFID index to detection points
rfid_detections = {i: [] for i in range(len(rfid_positions))}

# Define the robot's detection shape
def get_robot_detection_shape(position, orientation_degrees, robot_config, angle_resolution=360):
    x, y = position

    # Define the circle at the back
    circle_radius = robot_config.circle_radius
    back_circle = Point(x, y).buffer(circle_radius)

    # Define the front detection shape using the sinc function
    front_distance = robot_config.front_distance  # How far the sinc function extends
    angle_range = robot_config.angle_range  # Angular range for the front detection

    angles = np.linspace(-angle_range, angle_range, angle_resolution)

    # Rotate angles by the robot's orientation
    orientation_radians = np.deg2rad(orientation_degrees)
    rotated_angles = angles + orientation_radians

    sinc_values = np.sinc(angles / np.pi)  # Normalized sinc function

    # Scale sinc to desired front shape
    front_points = []
    for theta, s in zip(rotated_angles, sinc_values):
        distance = front_distance * s
        if distance <= 0:
            continue
        dx = distance * np.cos(theta)
        dy = distance * np.sin(theta)
        front_points.append((x + dx, y + dy))
    # Close the front shape by adding the robot position
    if front_points:
        front_polygon = Polygon([position] + front_points)
    else:
        front_polygon = Point(x, y)  # If no points, degenerate to a point

    # Combine the back circle and front shape
    detection_shape = back_circle.union(front_polygon)
    return detection_shape

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
        # Need to find orientation at this position
        # Find the corresponding orientation from the robot path
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

        # Calculate area
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

# Calculate overall RMSE
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
ax.set_xlim(0, room_width)
ax.set_ylim(0, room_height)
ax.set_aspect('equal')

# Plot RFIDs
for rfid_pos in rfid_positions:
    ax.plot(rfid_pos[0], rfid_pos[1], 'ro', label='RFID')

# Plot robot path
path_x = [pos[0] for pos in robot_path]
path_y = [pos[1] for pos in robot_path]
ax.plot(path_x, path_y, 'k--', linewidth=0.5, label='Robot Path')

# Function to animate the robot's movement
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
    if detection_patch is not None:
        detection_patch.remove()

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

# At the end, plot the estimated zones
for i, zone in rfid_zones.items():
    if zone and not zone.is_empty:
        x_zone, y_zone = zone.exterior.xy
        ax.fill(x_zone, y_zone, alpha=0.5, label=f'RFID {i} Estimated Zone')

ax.legend()
plt.title("RFID Localization Simulation")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.show()