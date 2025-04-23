import numpy as np
from shapely.geometry import Polygon

def detection_band_region(rssi_val, Pt, lam, G0, m,
                          x0=0, y0=0, theta0=0,
                          n_points=360, theta_lim=np.pi/2):
    """
    Return a single Shapely polygon for the region where
        floor = 10*floor(rssi_val/10)
        ceil  = 10*ceil(rssi_val/10)
    i.e., floor ≤ Pr_dBm < ceil, translated to (x0,y0) and rotated by theta0.

    Parameters
    ----------
    rssi_val : float
        Measured RSSI in dBm (e.g., 44).
    Pt, lam, G0, m : floats
        Same antenna parameters as detection_polygon.
    x0, y0 : float
        Antenna position in meters.
    theta0 : float
        Antenna orientation in radians.
    n_points : int
        Angular resolution for contour.
    theta_lim : float
        Half-angle of antenna main lobe.

    Returns
    -------
    shapely.geometry.Polygon or MultiPolygon
        The “band” region between the two threshold contours.
    """
    if rssi_val >=0 :
        return Polygon()
    
    # Determine thresholds
    upper = rssi_val + 2
    lower = rssi_val - 2

    # Compute the two bounding polygons
    poly_upper = detection_polygon(upper, Pt, lam, G0, m,
                                   x0=x0, y0=y0, theta0=theta0,
                                   n_points=n_points, theta_lim=theta_lim)
    poly_lower = detection_polygon(lower, Pt, lam, G0, m,
                                   x0=x0, y0=y0, theta0=theta0,
                                   n_points=n_points, theta_lim=theta_lim)
    
    # The band = lower region minus the inner upper region
    return poly_lower.difference(poly_upper)

def detection_polygon(threshold_dB, Pt, lam, G0, m,
                      x0=0, y0=0, theta0=0,
                      n_points=360, theta_lim=np.pi/2):
    """
    Return a Shapely polygon for the region where Pr_dB >= threshold_dB,
    positioned at (x0, y0) and rotated by theta0.

    Parameters
    ----------
    threshold_dB : float
        The RSSI threshold in dBm (e.g. 50 or 40).
    Pt : float
        Transmit power in watts.
    lam : float
        Wavelength in meters.
    G0 : float
        Peak linear antenna gain.
    m : float
        Cosine-power exponent (controls beamwidth).
    x0, y0 : float
        Antenna position in the plane (meters).
    theta0 : float
        Antenna boresight orientation (radians from +X axis).
    n_points : int
        Number of angular samples between -theta_lim…+theta_lim.
    theta_lim : float
        Half-angle of main lobe (default π/2 for ±90°).

    Returns
    -------
    shapely.geometry.Polygon
        Polygon where Pr_dB == threshold_dB, translated and rotated.
    """
    # Convert dBm threshold to linear Watts
    thr_lin = 10**(threshold_dB/10) 

    # Sample angles around boresight
    thetas = np.linspace(-theta_lim, theta_lim, n_points)
    # Antenna gain pattern G(θ) = G0 * cos^m(θ)
    G = G0 * np.cos(thetas)**m * (np.abs(thetas) <= theta_lim)

    # Invert Pr = Pt·G·(λ/(4πr))^2 ≥ thr_lin → r(θ) = ...
    r = (lam/(4*np.pi)) * np.sqrt(Pt * G / thr_lin)

    # Compute global angles and coordinates
    thetas_global = thetas + theta0
    xs = r * np.cos(thetas_global) + x0
    ys = r * np.sin(thetas_global) + y0

    return Polygon(zip(xs, ys))




def rssi_from_position(Pt, lam, G0, m, 
                       x_ant, y_ant, theta_ant,
                       x_recv, y_recv,
                       theta_lim=np.pi/2):
    """
    Reverse of detection range logic — compute RSSI from receiver position.

    Parameters
    ----------
    Pt : float
        Transmit power in watts.
    lam : float
        Wavelength in meters.
    G0 : float
        Peak antenna gain (linear).
    m : float
        Cosine-power exponent (controls beamwidth).
    x_ant, y_ant : float
        Antenna coordinates.
    theta_ant : float
        Antenna orientation angle (radians).
    x_recv, y_recv : float
        Receiver coordinates.
    theta_lim : float
        Beam width half-angle (radians). Default: π/2.

    Returns
    -------
    float
        Received RSSI in dBm, or -inf if outside antenna beam.
    """
    dx = x_recv - x_ant
    dy = y_recv - y_ant
    r = np.hypot(dx, dy)

    # Direction to receiver in global coordinates
    angle_to_recv = np.arctan2(dy, dx)

    # Relative angle from antenna boresight
    theta = angle_to_recv - theta_ant
    theta = (theta + np.pi) % (2 * np.pi) - np.pi  # wrap to [-π, π]

    if abs(theta) > theta_lim:
        return 0  # receiver is outside beam

    # Antenna gain at this angle
    G = G0 * np.cos(theta)**m

    # Received power (W)
    Pr = Pt * G * (lam / (4 * np.pi * r))**2

    # Convert to dBm
    return 10 * np.log10(Pr * 1e3)

def get_rssi_measurement(robot_pos, robot_orientation_deg, rfid_pos, robot_config):
    """
    Given the robot pose and an RFID position, compute the RSSI measurement.
    If the RFID is outside the robot's angular field-of-view, a zero value is returned.
    Otherwise, the theoretical RSSI is computed and Gaussian noise is added.
    """
    orientation_rad = np.deg2rad(robot_orientation_deg)

    ideal_rssi = rssi_from_position(robot_config.Pt, robot_config.lam, robot_config.G0, robot_config.m, 
                                    robot_pos[0], robot_pos[1], orientation_rad, rfid_pos[0], rfid_pos[1])
    rssi_noise = np.random.normal(0, robot_config.rssi_sigma)
    measured_rssi = ideal_rssi + rssi_noise - 30 # convert to db
    return measured_rssi