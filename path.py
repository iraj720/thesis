import numpy as np
from shapely.geometry import Point, Polygon, LineString

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

