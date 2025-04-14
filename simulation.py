import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from path import *

class RobotConfig:
    def __init__(self, circle_radius=1, front_distance=5, angle_range=np.pi/2):
        self.circle_radius = circle_radius
        self.front_distance = front_distance
        self.angle_range = angle_range

class RoomConfig:
    def __init__(self, room_width=50, room_height=10, horizontal_step=1, vertical_step=2):
        self.room_width = room_width
        self.room_height = room_height
        self.horizontal_step = horizontal_step
        self.vertical_step = vertical_step

def startSimulation(robot_config, room_config, rfid_positions, withAnimation):
    rfid_detections = {i: [] for i in range(len(rfid_positions))}
    shapes = {i: [] for i in range(len(rfid_positions))}

    rfid_zones = {}
    rfid_areas = {}
    rfid_rmse = {}

    robot_path = generate_robot_path(room_config.room_width, room_config.room_height, room_config.horizontal_step, room_config.vertical_step)

    for x, y, orientation in robot_path:
        position = (x, y)
        detection_shape = get_robot_detection_shape(position, orientation, robot_config)

        for i, rfid_pos in enumerate(rfid_positions):
            rfid_point = Point(rfid_pos)
            if detection_shape.contains(rfid_point):
                rfid_detections[i].append(position)
                shapes[i].append(detection_shape)

    for i, detections in rfid_detections.items():
        if shapes[i]:
            estimated_zone = shapes[i][0]
            for shape in shapes[i][1:]:
                estimated_zone = estimated_zone.intersection(shape)
            rfid_zones[i] = estimated_zone

            rfid_areas[i] = estimated_zone.area

            estimated_centroid = estimated_zone.centroid
            actual_position = Point(rfid_positions[i])
            error = estimated_centroid.distance(actual_position)
            rfid_rmse[i] = error
        else:
            # RFID was never detected
            rfid_zones[i] = None
            rfid_areas[i] = None
            rfid_rmse[i] = None

    if withAnimation:
        animate(room_config, robot_config, robot_path, rfid_zones, rfid_positions)
        
    return rfid_rmse, rfid_areas

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


def generate_robot_path(room_width, room_height, horizontal_step, vertical_step):
    path = []
    x = 0
    y = 0
    direction = 1 
    orientation = 0
    while y <= room_height:
        x_end = room_width if direction == 1 else 0
        while (direction == 1 and x <= x_end) or (direction == -1 and x >= x_end):
            path.append((x, y, orientation))
            x += horizontal_step * direction
        x = max(0, min(x, room_width))
        if y + vertical_step > room_height:
            break  # Do not proceed further if we've reached the max height
        if direction == 1:
            orientation = (orientation + 90) % 360
        else:
            orientation = (orientation - 90) % 360
        path.append((x, y, orientation))  # Include rotation at current position
        # Move up
        y += vertical_step
        path.append((x, y, orientation))  # Include movement up
        if direction == 1:
            orientation = (orientation + 90) % 360
        else:
            orientation = (orientation - 90) % 360
        path.append((x, y, orientation))  # Include rotation at current position
        direction *= -1
    return path




def animate(room_config, robot_config, robot_path, rfid_zones, rfid_positions):
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
        
        robot_marker.set_data([position[0]], [position[1]])

        # Plot detection shape
        detection_shape = get_robot_detection_shape(position, orientation, robot_config)
        if not detection_shape.is_empty and isinstance(detection_shape, Polygon):
            x_detection, y_detection = detection_shape.exterior.xy
            detection_patch = ax.fill(x_detection, y_detection, alpha=0.3, fc='blue')[0]
        else:
            detection_patch = None
        
        return robot_marker

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
