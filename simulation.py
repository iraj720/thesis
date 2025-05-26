import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

##############################
# Config Classes
##############################

class RobotConfig:
    def __init__(self, Pt=1.0, lam=0.33, G0=1.0, m=2.0, noise_db=1.0):
        self.Pt, self.lam, self.G0, self.m = Pt, lam, G0, m
        self.noise_db = noise_db

class RoomConfig:
    def __init__(self, x_max=10, y_max=10, z_max=5, step=1):
        self.x_max, self.y_max, self.z_max, self.step = x_max, y_max, z_max, step

##############################
# Antenna & Propagation
##############################

class IsotropicAntenna3D:
    def __init__(self, cfg: RobotConfig):
        self.Pt, self.lam, self.G0, self.m = cfg.Pt, cfg.lam, cfg.G0, cfg.m

    def measure_rssi(self, dist):
        # Friis: Pr ∝ d^{-m} → rssi_db = 10 log10(Pr)
        if dist <= 0 :
            return 1000
        
        Pr = self.Pt * self.G0 * (self.lam/(4*np.pi*dist))**self.m
        return 10*np.log10(Pr)

    def rssi_to_range(self, rssi_db):
        Pr = 10**(rssi_db/10)
        return self.lam/(4*np.pi)*(self.Pt*self.G0/Pr)**(1/self.m)

##############################
# Path Generation
##############################

def generate_robot_path_3d(room: RoomConfig):
    path = []
    # build a list of (x,y,z) waypoints in scan order
    waypoints = []
    zs = np.linspace(1, room.z_max-1, int(room.z_max/room.step))
    for z in zs:
        for xi in np.arange(0, room.x_max + 1e-6, room.step):
            for yi in np.arange(0, room.y_max + 1e-6, room.step):
                waypoints.append((xi, yi, z))

    # now compute a continuous path with orientations
    prev = None
    for wp in waypoints:
        x, y, z = wp
        if prev is None:
            # first point: we just point yaw=0, pitch=0
            yaw = 0.0
            pitch = 0.0
        else:
            dx = x - prev[0]
            dy = y - prev[1]
            dz = z - prev[2]
            # compute horizontal yaw
            yaw = np.degrees(np.arctan2(dy, dx))
            # compute pitch (elevation angle)
            horizontal_dist = np.hypot(dx, dy)
            if horizontal_dist < 1e-6:
                pitch = 0.0
            else:
                pitch = np.degrees(np.arctan2(dz, horizontal_dist))
        path.append((x, y, z, yaw, pitch))
        prev = (x, y, z)

    return path

##############################
# 3D Sector Volume
##############################

def build_sector_volume(center, yaw, pitch, rng, thickness=0.5, cone_ang=30):
    # spherical shell
    outer = trimesh.creation.icosphere(2, radius=rng+thickness)
    inner = trimesh.creation.icosphere(2, radius=max(rng-thickness,0.1))
    shell = outer.difference(inner)
    # cone
    cone = trimesh.creation.cone(radius=(rng+thickness)*np.tan(np.deg2rad(cone_ang)),
                                 height=2*(rng+thickness), sections=32)
    # align cone axis Z→ vector(yaw,pitch)
    # build rotation
    rot = R.from_euler('zy', [yaw, pitch], degrees=True).as_matrix()
    mat = np.eye(4); mat[:3,:3]=rot
    cone.apply_transform(mat)
    cone.apply_translation(center)
    vol = shell.intersection(cone)
    return vol

##############################
# Main Localization + Visualization
##############################

def localize_and_visualize(robot_cfg, room_cfg, tag_positions):
    ant = IsotropicAntenna3D(robot_cfg)
    path = generate_robot_path_3d(room_cfg)
    # measurements → volumes
    tag_volumes = {i: [] for i in range(len(tag_positions))}
    for pose in path:
        x,y,z,yaw,pitch = pose
        for i,(tx,ty,tz) in enumerate(tag_positions):
            d = np.linalg.norm([tx-x,ty-y,tz-z])
            rssi = ant.measure_rssi(d)
            rng = ant.rssi_to_range(rssi)
            vol = build_sector_volume((x,y,z), yaw, pitch, rng)
            tag_volumes[i].append(vol)

    # intersect all volumes per tag
    estimates, areas, errors = {}, {}, {}
    for i, vols in tag_volumes.items():
        if not vols:
            estimates[i]=None; areas[i]=0; errors[i]=None; continue
        region = vols[0]
        for v in vols[1:]:
            region = region.intersection(v)
            if region.is_empty: break
        if region.is_empty:
            estimates[i]=None; areas[i]=0; errors[i]=None
        else:
            c = region.centroid
            tx,ty,tz = tag_positions[i]
            err = np.linalg.norm([c[0]-tx, c[1]-ty, c[2]-tz])
            estimates[i]=(c,err)
            areas[i]=region.volume
            errors[i]=err

    # set up plotting
    fig = plt.figure(figsize=(12,5))
    ax_xy = fig.add_subplot(1,2,1); ax_xy.set_title("X–Y Projection")
    ax_xz = fig.add_subplot(1,2,2); ax_xz.set_title("X–Z Projection")
    for (tx,ty,tz) in tag_positions:
        ax_xy.plot(tx,ty,'ro'); ax_xz.plot(tx,tz,'ro')

    # animate robot and feasible region contours
    robot_dot_xy, = ax_xy.plot([],[],'b.')
    robot_dot_xz, = ax_xz.plot([],[],'b.')
    region_patch_xy=[]; region_patch_xz=[]

    def init():
        robot_dot_xy.set_data([],[]); robot_dot_xz.set_data([],[])
        return [robot_dot_xy,robot_dot_xz]

    def update(frame):
        # clear old
        for p in region_patch_xy+region_patch_xz:
            p.remove()
        region_patch_xy.clear(); region_patch_xz.clear()
        x,y,z,_,_ = path[frame]
        robot_dot_xy.set_data([x], [y])
        robot_dot_xz.set_data([x], [z])
        # draw region projections at this frame
        for i, vols in tag_volumes.items():
            if frame<len(vols):
                vol=vols[frame]
                if not vol.is_empty:
                    # mesh vertices
                    pts = np.array(vol.vertices)
                    # project XY
                    patch_xy = ax_xy.scatter(pts[:,0],pts[:,1],s=1,alpha=0.1)
                    # project XZ
                    patch_xz = ax_xz.scatter(pts[:,0],pts[:,2],s=1,alpha=0.1)
                    region_patch_xy.append(patch_xy)
                    region_patch_xz.append(patch_xz)
        return [robot_dot_xy,robot_dot_xz]+region_patch_xy+region_patch_xz

    ani = FuncAnimation(fig, update, frames=len(path),
                        init_func=init,interval=200,blit=False)
    plt.tight_layout()
    plt.show()

    # print summary
    print("Estimates:", estimates)
    print("Areas:", areas)
    print("Errors:", errors)

if __name__=="__main__":
    rc = RobotConfig()
    rm = RoomConfig()
    tags=[(5,5,1),(15,10,3)]
    localize_and_visualize(rc, rm, tags)
