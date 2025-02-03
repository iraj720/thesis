from simulation import *

# RFID positions
rfid_positions = [
    (10.1, 5.5),
    (25.1, 7.3),
    (40.3, 3.3),
    (35.6, 9.1),
    (15.6, 1)
]

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

robot_config = RobotConfig(circle_radius=1, front_distance=3, angle_range=np.pi/4)
room_config = RoomConfig()

if __name__ == "__main__":
    startSimulation(robot_config, room_config, rfid_positions)