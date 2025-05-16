from antenna import *
from datasets import *
from math import *
from shapely.geometry import Polygon, MultiPolygon, box, Point, LineString
from shapely.ops import unary_union
from shapely.affinity import rotate, translate
import matplotlib.pyplot as plt
import numpy as np


def detection_shape(azimuth, rssi, estimated_noise_in_percent, antenna: Antenna):
    if rssi >= 0:
        return Polygon()

    lower = rssi * (100 - estimated_noise_in_percent) / 100
    upper = rssi * (100 + estimated_noise_in_percent) / 100

    upper_detection_shape = antenna.detection_shape(upper, azimuth)
    lower_detection_shape = antenna.detection_shape(lower, azimuth)

    return upper_detection_shape.difference(lower_detection_shape)


def estimated_zone(detected_shapes : list):
    estimated_zone = detected_shapes[0]
    for s in detected_shapes[1:]:
        intersection = estimated_zone.intersection(s)
        if not intersection.is_empty:
            estimated_zone = intersection

    return estimated_zone

def subdivide_to_squares(bbox, m):
    """
    Given a bounding box (minx, miny, maxx, maxy) and an integer m,
    return at least m equal-sized "square" cells covering that box.
    We form an n*n grid where n=ceil(sqrt(m)), then return the first m cells.
    """
    minx, miny, maxx, maxy = bbox
    width, height = maxx - minx, maxy - miny

    n = ceil(sqrt(m))

    a = width/floor(sqrt(m))
    b = height/floor(sqrt(m))

    cells = []
    for i in range(n):
        for j in range(n):
            x0 = minx + i * a
            y0 = miny + j * b
            x1 = x0 + a
            y1 = y0 + b
            cells.append(box(x0, y0, x1, y1))
            
    
    return cells  # fallback

def find_deepest_intersection_cell(polygons, m=4, max_iters=50):
    """
    Iteratively subdivide the union-bbox into m squares, pick the one
    with the most polygon intersections, and repeat until only one cell
    has a non-zero intersection count among its siblings.
    """
    # 1. Compute the global union-bbox
    union_poly = unary_union(polygons)
    current_bbox = union_poly.bounds  # (minx, miny, maxx, maxy)
    print("boxx: ", len(polygons))

    for iteration in range(max_iters):
        # 2. Subdivide into (at least) m squares
        cells = subdivide_to_squares(current_bbox, m)
        print(current_bbox)

        # 3. Count intersections
        counts = [sum(1 for p in polygons if p.intersects(cell)) for cell in cells]
        print(counts)

        # if max(counts) < len(polygons)/2:
        #     break

        # 4. Check how many cells have non-zero intersections
        nonzero_idxs = [i for i, c in enumerate(counts) if c == len(polygons)]
        if len(nonzero_idxs) >= m:
            if nonzero_idxs:
                break

        # 5. Otherwise pick the cell with the highest count
        max_count = max(counts)
        best_indices = [i for i, count in enumerate(counts) if count == max_count]
        best_cells = [cells[i] for i in best_indices]
        union_cell = unary_union(best_cells)
        current_bbox = union_cell.bounds

    return box(*current_bbox)

if __name__ == "__main__":
    
    edge_data, edges_df = bbil_dataset()

    detected_shapes = []
    for rec in edge_data["edge_1"]:
        x, y, rssi, azimuth = rec["realx"], rec["realy"], rec["rssi"], rec["azimuth"]
        detected_shape = detection_shape(((90-azimuth)*np.pi)/180 ,rssi, 12, IsotropicAntenna(), x, y)
        detected_shape = translate(detected_shape, xoff=x, yoff=y)
        if detected_shape.is_valid:
            detected_shapes.append(detected_shape)


    # detected_shapes = [find_deepest_intersection_cell(detected_shapes, m=4, max_iters=i) for i in range(10)]

    res1 = find_deepest_intersection_cell(detected_shapes, m=4, max_iters=20)
    res2 = estimated_zone(detected_shapes)

    x11, y11 = res1.centroid.x, res1.centroid.y
    x1, y1 = res2.centroid.x, res2.centroid.y
    x2, y2 = edges_df["edge_x"][0], edges_df["edge_y"][0]

    print(res2.centroid)
    print(res1.centroid)

    # Compute RMSE
    rmse1 = sqrt((x11 - x2)**2 + (y11 - y2)**2)
    rmse2 = sqrt((x1 - x2)**2 + (y1 - y2)**2)

    print("RMSE : ", rmse2)
    print("RMSE : ", rmse1)

    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()

    for res in detected_shapes:
        if not res.is_empty:
            if res.geom_type == 'Polygon':
                x_ext, y_ext = res.exterior.xy
                ax.fill(x_ext, y_ext, alpha=0.5)

                for interior in res.interiors:
                    x_int, y_int = interior.xy
                    ax.fill(x_int, y_int, fc='white')

            elif res.geom_type == 'MultiPolygon':
                for poly in res.geoms:
                    x, y = poly.exterior.xy
                    ax.fill(x, y, alpha=0.5, ec='black')

        ax.set_title("Cumulative Detection Shapes")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_aspect('equal')
        ax.grid(True)

        plt.draw()
        plt.pause(0.5)

    plt.ioff()
    plt.show()
