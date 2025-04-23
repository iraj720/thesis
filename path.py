def generate_robot_path(room_width, room_height, horizontal_step, vertical_step):
    path = []
    x, y = 0, 0
    direction = 1  # 1 for right, -1 for left
    orientation = 0  # Degrees

    while y <= room_height:
        # Move horizontally
        x_end = room_width if direction == 1 else 0
        while (direction == 1 and x <= x_end) or (direction == -1 and x >= x_end):
            path.append((x, y, orientation))
            x += direction * horizontal_step
        x = max(0, min(x, room_width))  # Clamp x within bounds

        # Stop if next vertical step exceeds room height
        if y + vertical_step > room_height:
            break

        # Rotate 90°, move up, rotate 90° again
        orientation = (orientation + direction * 90) % 360
        path.append((x, y, orientation))  # Rotation at edge

        y += vertical_step
        path.append((x, y, orientation))  # Move up

        orientation = (orientation + direction * 90) % 360
        path.append((x, y, orientation))  # Second rotation

        direction *= -1  # Reverse horizontal direction

    return path

