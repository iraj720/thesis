import numpy as np
import math
from shapely.geometry import Polygon, Point, MultiPolygon
from scipy.stats import beta as BetaDist
from abc import ABC, abstractmethod
from estimate_std import *

# === Abstract Antenna Interface === #
class Antenna(ABC):
    @abstractmethod
    def detection_shape(self, rssi, azimuth) -> Polygon:
        """Return detection region as a polygon based on RSSI and azimuth."""
        pass

    def estimated_noise_in_sigma(self, rssi) -> float:
        """Estimate noise level in sigma based on RSSI value."""
        return estimated_std_poly(rssi)


# === Custom Sector-Based Antenna === #
class CustomAntenna(Antenna):
    def detection_shape(self, rssi, azimuth, angle_range=90) -> Polygon:
        if rssi >= 0 or rssi <= -100:
            return Polygon()  # Invalid signal strength

        num_points = 64  # Angular resolution
        half_angle = azimuth - (angle_range / 2)
        points = [(0, 0)]  # Start with antenna position at origin

        # detection_shape first considers n number of angles and for each angle it computes the correspondig d for it
        # so we have d with phase of theta which is a point 
        # by putting all of these points together we have the final detection shape

        for i in range(num_points + 1):
            theta = half_angle + (i * angle_range / num_points)
            phi = (i * angle_range / num_points) - (angle_range / 2)

            # Distance model as an exponential function of RSSI and phi
            d = np.exp((-rssi - (phi ** 2) / 227 - 42.43) / 11.55)
            rad = np.deg2rad(theta)
            x = d * math.cos(rad)
            y = d * math.sin(rad)
            points.append((x, y))

        return Polygon(points)

    def measure_rssi(self, x, y, x_ant, y_ant, azimuth):
        """Measure RSSI at (x, y) relative to antenna at (x_ant, y_ant)."""
        angleAtt = self.angleAttenuation(x, y, x_ant, y_ant, azimuth)
        if angleAtt is None:
            return -1000  # Out of beam
        distance = np.hypot(x_ant - x, y_ant - y)
        return calculate_rssi(distance) + angleAtt

    def angleAttenuation(self, x, y, x_ant, y_ant, azimuth):
        """Return angle-dependent attenuation if within 45°, otherwise None."""
        dx, dy = x - x_ant, y - y_ant
        angle_to_point = np.degrees(np.arctan2(dy, dx))
        delta_theta = (angle_to_point - azimuth + 180) % 360 - 180

        if abs(delta_theta) > 45:
            return None
        return -(delta_theta ** 2) / 227

class CosineAntenna(Antenna):
    def __init__(self,Pt=1, lam=0.33, G0=2.0, m=4, n_points=360, theta_lim=np.pi/2):
        self.Pt=Pt
        self.lam=lam
        self.G0=G0
        self.m=m
        self.n_points=n_points
        self.theta_lim=theta_lim

    def measure_rssi(self, x, y, x_ant, y_ant, azimuth): 
        dx = x - x_ant
        dy = y - y_ant
        r = np.hypot(dx, dy)

        # Direction to receiver in global coordinates
        angle_to_recv = np.arctan2(dy, dx)

        # Relative angle from antenna boresight
        theta = angle_to_recv - azimuth
        theta = (theta + np.pi) % (2 * np.pi) - np.pi  # wrap to [-π, π]

        if abs(theta) > self.theta_lim:
            return 0  # receiver is outside beam

        # Antenna gain at this angle
        G = self.G0 * np.cos(theta)**self.m

        # Received power (W)
        Pr = self.Pt * G * (self.lam / (4 * np.pi * r))**2

        # Convert to dBm
        return 10 * np.log10(Pr)

    # 5. Beta-distribution pattern
    def detection_shape(self, rssi, azimuth):
        # Convert dBm threshold to linear Watts
        thr_lin = 10**(rssi/10) 

        # Sample angles around boresight
        thetas = np.linspace(-self.theta_lim, self.theta_lim, self.n_points)
        # Antenna gain pattern G(θ) = G0 * cos^m(θ)
        G = self.G0 * np.cos(thetas)**self.m * (np.abs(thetas) <= self.theta_lim)

        # Invert Pr = Pt·G·(λ/(4πr))^2 ≥ thr_lin → r(θ) = ...
        r = (self.lam/(4*np.pi)) * np.sqrt(self.Pt * G / thr_lin)

        # Compute global angles and coordinates
        thetas_global = thetas + azimuth
        xs = r * np.cos(thetas_global)
        ys = r * np.sin(thetas_global)

        return Polygon(zip(xs, ys))
    

class SincAntenna(Antenna):
    # 5. Beta-distribution pattern
    def detection_shape(self, rssi: float, azimuth: float, alpha: float = 5.0, n: float = 2.0,
                    scale: float = 1.0, num_points: int = 360) -> Polygon:
        
        if rssi >= 0 or rssi <= -100:
            return Polygon()

        pr = rssi_to_linear(rssi)
        thetas = np.linspace(0, 2 * np.pi, num_points)
        diffs = thetas - np.deg2rad(azimuth)
        # avoid divide by zero
        x = alpha * diffs
        gains = np.sinc(x / np.pi) ** n  # numpy sinc is sin(pi*x)/(pi*x)
        rs = scale * pr * gains
        points = [(r * np.cos(theta), r * np.sin(theta)) for r, theta in zip(rs, thetas)]
        return Polygon(points)
    
class BetaAntenna(Antenna):
    # Beta-distribution pattern tuned for room-scale detection
    def detection_shape(self, rssi: float, azimuth: float, alpha: float = 2.5, beta_param: float = 5.0,
                        scale: float = 150.0, num_points: int = 360) -> Polygon:
        if rssi >= 0 or rssi <= -100:
            return Polygon()

        pr = rssi_to_linear(rssi)  # Assumes exponential scale, like 10**(rssi/10)
        thetas = np.linspace(0, np.pi, num_points)  # Half-plane
        x = thetas / np.pi
        gains_half = BetaDist.pdf(x, alpha, beta_param)
        gains = np.concatenate([gains_half, gains_half[::-1]])  # Full 360°
        thetas_full = np.linspace(0, 2 * np.pi, num_points * 2)
        thetas_full = thetas_full + np.deg2rad(azimuth)
        rs = scale * pr * gains
        points = [(r * np.cos(theta), r * np.sin(theta)) for r, theta in zip(rs, thetas_full)]
        return Polygon(points)


def calculate_rssi(distance_meters):
    if distance_meters <= 0:
        raise ValueError("Distance must be greater than 0 meters.")
    
    return -11.55 * math.log(distance_meters) - 42.43

# Utility: convert RSSI (dBm) to linear scale (power ratio)
def rssi_to_linear(rssi_dbm: float) -> float:
    # Assuming 0 dBm -> 1.0, adjust if needed
    return 10 ** (rssi_dbm / 10.0)


# # 1. Von Mises pattern
# def von_mises_polygon(rssi_dbm: float, azimuth: float, kappa: float = 4.0,
#                       scale: float = 1.0, num_points: int = 360) -> Polygon:
#     pr = rssi_to_linear(rssi_dbm)
#     thetas = np.linspace(0, 2 * np.pi, num_points)
#     diffs = thetas - np.deg2rad(azimuth)
#     gains = np.exp(kappa * np.cos(diffs))
#     rs = scale * pr * gains
#     points = [(r * np.cos(theta), r * np.sin(theta)) for r, theta in zip(rs, thetas)]
#     return Polygon(points)

# # 2. Gaussian pattern
# def gaussian_polygon(rssi_dbm: float, azimuth: float, sigma: float = np.pi/8,
#                      scale: float = 1.0, num_points: int = 360) -> Polygon:
#     pr = rssi_to_linear(rssi_dbm)
#     thetas = np.linspace(0, 2 * np.pi, num_points)
#     diffs = thetas - np.deg2rad(azimuth)
#     gains = np.exp(-0.5 * (diffs / sigma) ** 2)
#     rs = scale * pr * gains
#     points = [(r * np.cos(theta), r * np.sin(theta)) for r, theta in zip(rs, thetas)]
#     return Polygon(points)

# # 4. Generalized cardioid / Fourier series
# def cardioid_polygon(rssi_dbm: float, azimuth: float, beta: float = 0.5,
#                      scale: float = 1.0, num_points: int = 360) -> Polygon:
#     pr = rssi_to_linear(rssi_dbm)
#     thetas = np.linspace(0, 2 * np.pi, num_points)
#     diffs = thetas - np.deg2rad(azimuth)
#     gains = 1 + beta * np.cos(diffs)
#     rs = scale * pr * gains
#     points = [(r * np.cos(theta), r * np.sin(theta)) for r, theta in zip(rs, thetas)]
#     return Polygon(points)

# def detection_band_region(rssi_val, Pt, lam, G0, m,
#                           x0=0, y0=0, theta0=0,
#                           n_points=360, theta_lim=np.pi/2):
#     """
#     Return a single Shapely polygon for the region where
#         floor = 10*floor(rssi_val/10)
#         ceil  = 10*ceil(rssi_val/10)
#     i.e., floor ≤ Pr_dBm < ceil, translated to (x0,y0) and rotated by theta0.

#     Parameters
#     ----------
#     rssi_val : float
#         Measured RSSI in dBm (e.g., 44).
#     Pt, lam, G0, m : floats
#         Same antenna parameters as detection_polygon.
#     x0, y0 : float
#         Antenna position in meters.
#     theta0 : float
#         Antenna orientation in radians.
#     n_points : int
#         Angular resolution for contour.
#     theta_lim : float
#         Half-angle of antenna main lobe.

#     Returns
#     -------
#     shapely.geometry.Polygon or MultiPolygon
#         The “band” region between the two threshold contours.
#     """
#     if rssi_val >=0 :
#         return Polygon()
    
#     # Determine thresholds
#     upper = rssi_val + 2
#     lower = rssi_val - 2

#     # Compute the two bounding polygons
#     poly_upper = detection_polygon(upper, Pt, lam, G0, m,
#                                    x0=x0, y0=y0, theta0=theta0,
#                                    n_points=n_points, theta_lim=theta_lim)
#     poly_lower = detection_polygon(lower, Pt, lam, G0, m,
#                                    x0=x0, y0=y0, theta0=theta0,
#                                    n_points=n_points, theta_lim=theta_lim)
    
#     # The band = lower region minus the inner upper region
#     return poly_lower.difference(poly_upper)

# def detection_polygon(threshold_dB, Pt, lam, G0, m,
#                       x0=0, y0=0, theta0=0,
#                       n_points=360, theta_lim=np.pi/2):
#     """
#     Return a Shapely polygon for the region where Pr_dB >= threshold_dB,
#     positioned at (x0, y0) and rotated by theta0.

#     Parameters
#     ----------
#     threshold_dB : float
#         The RSSI threshold in dBm (e.g. 50 or 40).
#     Pt : float
#         Transmit power in watts.
#     lam : float
#         Wavelength in meters.
#     G0 : float
#         Peak linear antenna gain.
#     m : float
#         Cosine-power exponent (controls beamwidth).
#     x0, y0 : float
#         Antenna position in the plane (meters).
#     theta0 : float
#         Antenna boresight orientation (radians from +X axis).
#     n_points : int
#         Number of angular samples between -theta_lim…+theta_lim.
#     theta_lim : float
#         Half-angle of main lobe (default π/2 for ±90°).

#     Returns
#     -------
#     shapely.geometry.Polygon
#         Polygon where Pr_dB == threshold_dB, translated and rotated.
#     """
#     # Convert dBm threshold to linear Watts
#     thr_lin = 10**(threshold_dB/10) 

#     # Sample angles around boresight
#     thetas = np.linspace(-theta_lim, theta_lim, n_points)
#     # Antenna gain pattern G(θ) = G0 * cos^m(θ)
#     G = G0 * np.cos(thetas)**m * (np.abs(thetas) <= theta_lim)

#     # Invert Pr = Pt·G·(λ/(4πr))^2 ≥ thr_lin → r(θ) = ...
#     r = (lam/(4*np.pi)) * np.sqrt(Pt * G / thr_lin)

#     # Compute global angles and coordinates
#     thetas_global = thetas + theta0
#     xs = r * np.cos(thetas_global) + x0
#     ys = r * np.sin(thetas_global) + y0

#     return Polygon(zip(xs, ys))