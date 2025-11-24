import os
import re
import numpy as np

def parse_speed_from_filename(filename):
    """
    Extract speed from filenames like:
    'CAT V2_T1-15_5 m_s-VLM1.txt'
    """
    match = re.search(r"(\d+[.,_]\d+)\s*m[_ ]s", filename)
    if not match:
        return None

    number = match.group(1).replace("_", ".")
    return float(number)


def load_polar_file(path):
    """
    Reads an XFLR5 VLM polar file and extracts:

    alpha, CL, CD (total)
    """
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


def get_cl_cd(speed, alpha, folder="mission_script/prop"):
    """
    Returns interpolated CL and CD for the given (speed, alpha).

    - Finds the file whose encoded speed is closest to `speed`
    - Loads and interpolates over alpha
    """
    if not os.path.isdir(folder):
        raise RuntimeError(f"Polar folder not found: {folder}")

    files = [f for f in os.listdir(folder) if f.endswith(".txt")]

    speed_map = {}
    for f in files:
        s = parse_speed_from_filename(f)
        if s is not None:
            speed_map[f] = s

    if not speed_map:
        raise RuntimeError("No valid speed-encoded files found in XFLR_Values.")

    # snap to closest speed
    best_file = min(speed_map, key=lambda fn: abs(speed_map[fn] - speed))
    best_speed = speed_map[best_file]

    path = os.path.join(folder, best_file)
    alphas, cls, cds = load_polar_file(path)

    if len(alphas) == 0:
        raise RuntimeError(f"No aerodynamic data found in: {path}")

    cl = np.interp(alpha, alphas, cls)
    cd = np.interp(alpha, alphas, cds)

    return cl, cd, best_speed, best_file
