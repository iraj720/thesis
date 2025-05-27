import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# Data
rssi = np.array([-40, -45, -50, -55, -60, -65, -70, -75, -80])
std = np.array([0.2068, 0.2109, 0.2197, 0.2389, 0.2805, 0.3705, 0.5647, 0.9853, 1.8953])

coeffs = np.polyfit(rssi, std, deg=4)
# e.g. coeffs = [a, b, c, d]

def estimated_std_poly(rssi: float) -> float:
    a, b, c, d, e  = coeffs
    return  a*rssi**4 + b*rssi**3 + c*rssi**2 + d*rssi + e

def angleAttenuation( x, y, x_ant, y_ant, azimuth):
    # Vector from antenna to point
    dx = x - x_ant
    dy = y - y_ant
    
    # Angle from antenna to point (in degrees)
    angle_to_point = np.degrees(np.arctan2(dy, dx))
    
    # Normalize angles to [-180, 180]
    delta_theta = (angle_to_point - azimuth + 180) % 360 - 180
    
    # Optional: Apply attenuation based on angle difference
    # Example: Quadratic attenuation (penalize wide angles)
    attenuation = -(delta_theta ** 2) / 227  # original scale you used

    return delta_theta

if __name__ == "__main__":
    
    print(angleAttenuation(3,3, 6, 0, 0))