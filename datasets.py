import numpy as np
from shapely.affinity import rotate, translate
from matplotlib.patches import Polygon as MplPolygon
import pandas as pd
import glob
import os
from bisect import bisect_left
from datetime import datetime

 # 2) Define interpolator
def get_azimuth_at(t_query: datetime, df) -> float:
    """Return linearly interpolated azimuth at datetime t_query."""
    # Extract numpy arrays of times (as floats: seconds since epoch) and azimuths
    times = df['Datetime'].astype(np.int64) / 1e9  # convert ns → s
    azs   = df['azimuth'].values

    tq = t_query.timestamp()  # seconds since epoch
    # find rightmost index where times[idx] < tq
    idx = bisect_left(times, tq)
    if idx == 0:
        return float(azs[0])
    if idx >= len(times):
        return float(azs[-1])
    t0, t1 = times[idx-1], times[idx]
    a0, a1 = azs[idx-1], azs[idx]
    # linear interp
    return float(a0 + (a1 - a0)*( (tq - t0) / (t1 - t0) ))

def bbil_dataset():
    folder_path = "./experiment/experiment1/test/"

    edges_df = pd.read_csv("./experiment/experiment1/edges.csv")

    # 2. Build a list of all matching filenames
    pattern = os.path.join(folder_path, "*1_data_wide.csv")
    patternAzimuth = os.path.join(folder_path, "*1_com.csv")

    file_list_azimuth = glob.glob(patternAzimuth)
    file_list = glob.glob(pattern)

    # 3. Read and append
    df_list = []
    for file in file_list:
        df = pd.read_csv(file, parse_dates=["Datetime"])  # parse your timestamp if present
        df_list.append(df)

    # 4. Concatenate into one DataFrame
    all_data = pd.concat(df_list, ignore_index=True)

    # 3. Read and append
    df_list_azimuth = []
    for file in file_list_azimuth:
        df = pd.read_csv(file, parse_dates=["Datetime"])  # parse your timestamp if present
        df_list_azimuth.append(df)

    # 4. Concatenate into one DataFrame
    all_data = pd.concat(df_list, ignore_index=True)
    all_data_azimuth = pd.concat(df_list_azimuth, ignore_index=True)
    all_data_azimuth = all_data_azimuth.sort_values('Datetime').reset_index(drop=True)


    # 5. (Optional) Inspect
    print(f"Read {len(file_list)} files, total rows = {len(all_data)}")
    print(all_data.head())

    print(f"Read {len(file_list_azimuth)} files, total rows = {len(all_data_azimuth)}")
    print(all_data_azimuth.head())

    edge_cols = [col for col in all_data.columns if col.startswith("edge_")]

    edge_data = {}
    for edge in edge_cols:
        # select only realx, realy, and this edge’s RSSI
        sub = all_data[["Datetime","realx", "realy", edge]].copy()
        # rename the RSSI column to a common name
        sub = sub.rename(columns={edge: "rssi"})
        # convert to list of dicts (or you could keep it as a DataFrame)
        edge_data[edge] = sub.to_dict(orient="records")

    for rec in edge_data["edge_1"]:
        rec["azimuth"] = get_azimuth_at(rec["Datetime"], all_data_azimuth)

    return edge_data, edges_df