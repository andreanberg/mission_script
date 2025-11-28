import os
import re
import numpy as np
import pandas as pd
import xlrd

def file_speed(filename):
    match = re.search(r"(\d+[.,_]\d+)\s*m[_ ]s", filename)
    if not match:
        return None

    number = match.group(1).replace("_", ".")
    return float(number)


def load_file(path):
    alphas, cls, cds = [], [], []
    with open(path, "r") as f:
        for line in f:
            parts = line.split()
            if len(parts) < 6:
                continue
            try:
                a = float(parts[0])
                Cl = float(parts[2])
                Cd = float(parts[5])
                alphas.append(a)
                cls.append(Cl)
                cds.append(Cd)
            except ValueError:
                continue
    return np.array(alphas), np.array(cls), np.array(cds)


def get_cl_cd(speed, alpha, folder="prop/aero"):
    if not os.path.isdir(folder):
        raise RuntimeError(f"Folder not found: {folder}")
    files = [f for f in os.listdir(folder) if f.endswith(".txt")]
    speed_map = {}
    for f in files:
        s = file_speed(f)
        if s is not None:
            speed_map[f] = s

    if not speed_map:
        raise RuntimeError("No valid speed-encoded files found.")

    # snap to closest speed
    best_file = min(speed_map, key=lambda fn: abs(speed_map[fn] - speed))
    best_speed = speed_map[best_file]
    
    path = os.path.join(folder, best_file)
    alphas, cls, cds = load_file(path)
    
    if len(alphas) == 0:
        raise RuntimeError(f"No aerodynamic data found in: {path}")

    cl = np.interp(alpha, alphas, cls)
    cd = np.interp(alpha, alphas, cds)

    return cl, cd, best_speed, best_file

# TODO make this readible and read from it

#file = "prop/motor/testParameter_AT4125 VTOL Fixed Wing Aircraft Long Shaft Motor KV250_KV540.xls"
#df = pd.read_excel(file, engine="xlrd")

"""
# If xlrd is not available, convert your file to .xlsx manually,
# then simply use:
# df = pd.read_excel("motor_test.xlsx")

# --- Clean column names (optional but recommended) ---
df.columns = df.columns.str.strip()

# --- Extract arrays ---
throttle_pct = df["Throttle"].str.replace("%", "").astype(float).values  
voltage      = df["Voltage (V)"].astype(float).values
current      = df["Current (A)"].astype(float).values
power        = df["Power (W)"].astype(float).values
rpm          = df["RPM"].astype(float).values
thrust_g     = df["Thrust (g)"].astype(float).values

# Convert thrust to Newtons
thrust_N = thrust_g * 9.81 / 1000.0

# --- Optional interpolation functions ---
throttle_norm = throttle_pct / 100.0

thrust_fn  = lambda t: np.interp(t, throttle_norm, thrust_N)
power_fn   = lambda t: np.interp(t, throttle_norm, power)
rpm_fn     = lambda t: np.interp(t, throttle_norm, rpm)

# Print summaries
print("Throttle (%):", throttle_pct)
print("Thrust (N):", thrust_N)
print("RPM:", rpm)
print("Power (W):", power)
print("Voltage:", voltage)
print("Current:", current)
"""