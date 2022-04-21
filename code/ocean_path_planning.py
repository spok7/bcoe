import numpy as np
from dataset_processing.soda_dataloader import SODA

soda = SODA()


def get_flow_vel(x, y, z):
    # function: whirlpool
    # vx = -yz
    # vy = xz
    # vz = 0
    # function: current
    # vx = z - 100
    # vy = 0
    # vz = 0
    vx = z
    vy = 0
    vz = 0
    return soda.query([x, y, z])


def ocean_path_planning(sx, sy, sz, rvx, rvy, rvz, delta_t, step_size=0.1):
    # ts = np.linspace(0, delta_t, np.ceil(delta_t / step_size))
    npts = 100
    ts = np.linspace(0,delta_t, npts)
    dt = ts[1] - ts[0]
    px = [sx]
    py = [sy]
    pz = [sz]
    x = sx
    y = sy
    z = sz
    for t in ts:
        vx, vy, vz = get_flow_vel(x, y, z)
        x += dt * (vx + rvx)
        y += dt * (vy + rvy)
        z += dt * (vz + rvz)
        px.append(x)
        py.append(y)
        pz.append(z)
    return px, py, pz, 0, delta_t
