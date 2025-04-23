##########################
# 📽️ RFID Animation Logic
##########################

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from shapely.geometry import Polygon, Point, GeometryCollection, LineString, MultiPolygon
import numpy as np
import time

###############################################
# 🎞️ Main Animation Function
###############################################
def animate(room_config, robot_config, robot_path, rfid_zones, rfid_positions, all_shapes: dict):
    # Setup plot
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.set_xlim(0, room_config.room_width)
    ax.set_ylim(0, room_config.room_height)
    ax.set_aspect('equal')
    ax.set_title("RSSI-Based RFID Localization")
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")

    # Plot static RFID tag positions
    for rfid_pos in rfid_positions:
        ax.plot(rfid_pos[0], rfid_pos[1], 'ro', label='RFID')

    # Plot robot's full path (dashed line)
    path_x = [pos[0] for pos in robot_path]
    path_y = [pos[1] for pos in robot_path]
    ax.plot(path_x, path_y, 'k--', linewidth=0.5, label='Robot Path')

    # Initialize robot marker (blue dot)
    robot_marker, = ax.plot([], [], 'bo', label='Robot')

    # Initialize estimated zones and patch tracking
    estimated_zones = {rfid: shapes[0] for rfid, shapes in all_shapes.items()}
    zone_patches = []

    ##################
    # 🎬 Init Function
    ##################
    def init():
        robot_marker.set_data([], [])
        return [robot_marker]

    #####################
    # 🔄 Frame-by-frame Update
    #####################
    def update(frame):
        # Remove previous visualization patches
        for patches in zone_patches:
            for patch in patches:
                patch.remove()
        zone_patches.clear()

        # Update robot marker position
        x, y, _ = robot_path[frame]
        robot_marker.set_data([x], [y])

        # For each RFID, compute updated estimated zone
        for rfid_id, shapes in all_shapes.items():
            estimated_zone = estimated_zones[rfid_id].intersection(shapes[frame])
            if not estimated_zone.is_empty:
                estimated_zones[rfid_id] = estimated_zone

            patches = []

            # Fill area if intersection exists
            if not estimated_zones[rfid_id].is_empty and hasattr(estimated_zone, 'exterior'):
                if isinstance(estimated_zones[rfid_id], Polygon):
                    x, y = estimated_zones[rfid_id].exterior.xy
                    patches = ax.fill(x, y, alpha=0.5, label=f'RFID {rfid_id} Estimated Zone')
                elif isinstance(estimated_zones[rfid_id], MultiPolygon):
                    for poly in estimated_zones[rfid_id].geoms:
                        x, y = poly.exterior.xy
                        patches_temp = ax.fill(x, y, alpha=0.5, label=f'RFID {rfid_id} Estimated Zone')
                        patches.extend(patches_temp)
                zone_patches.append(patches)

        time.sleep(0.1)  # For slower, more visual animation
        return [robot_marker] + [p for sub in zone_patches for p in sub]

    ####################
    # ▶️ Run Animation
    ####################
    ani = FuncAnimation(
        fig, update, frames=len(robot_path),
        init_func=init, blit=False, interval=100, repeat=False
    )

    # Show static estimated zones after animation
    for i, zone in rfid_zones.items():
        if zone and not zone.is_empty:
            plot_shape(ax, zone, alpha=0.5, label=f'RFID {i} Estimated Zone')

    ax.legend()
    plt.title("RFID Localization Simulation")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.show()


######################################
# 🧩 Shape Plotting Utility
######################################
def plot_shape(ax: plt.Axes, shape, **kwargs):
    if isinstance(shape, Polygon):
        x, y = shape.exterior.xy
        ax.fill(x, y, **kwargs)
    elif isinstance(shape, MultiPolygon):
        for poly in shape.geoms:
            x, y = poly.exterior.xy
            ax.fill(x, y, **kwargs)
    elif isinstance(shape, LineString):
        x, y = shape.xy
        ax.fill(x, y, **kwargs)
    elif isinstance(shape, GeometryCollection):
        for geom in shape.geoms:
            plot_shape(ax, geom, **kwargs)
    else:
        print("⚠️ Unhandled geometry type:", type(shape))
