import numpy as np
import casadi as cs
import os


def getIPOPTData(N: int,
                 Tf: float,
                 offset: float,
                 TRACK: dict):
    """Extracts mintime IPOPT optimization results, which can be generated using the repository
    `global_racetrajectory_optimization`.

    :param N: number of discretization steps of the track
    :param Tf: final optimization horizon
    :param offset: offset not to be considered in the IPOPT data
    :param TRACK: track parameters (length, number of discretization steps per lap)

    :Authors:
        Maximilian Bayerlein

    :Created on:
        01.08.2020
    """

    file_path = os.path.dirname(__file__)
    file_path = os.path.join(file_path, '../inputs', TRACK["Name"], 'mintime')

    states = np.genfromtxt(os.path.join(file_path, 'states.csv'), delimiter=';', skip_header=1)
    controls = np.genfromtxt(os.path.join(file_path, 'controls.csv'), delimiter=';', skip_header=1) * 0.001
    controls = np.vstack((controls, controls[-1, :]))  # enlarge control array to length of states array

    s = states[:, 0]        # [m]
    v = states[:, 2]        # [m/s]
    t = states[:, 1]        # [s]
    soc = states[:, 12]     # [%]
    tb = states[:, 8]       # [°C]
    tm = states[:, 7]       # [°C]
    ti = states[:, 9]       # [°C]
    tcmi = states[:, 10]    # [°C]
    tcb = states[:, 11]     # [°C]
    fd = controls[:, 3]     # [kN]
    fb = controls[:, 4]     # [kN]

    # interpolate states in terms of position points
    v_interp = cs.interpolant('v_interp', 'bspline', [s], v)
    t_interp = cs.interpolant('t_interp', 'bspline', [s], t)
    soc_interp = cs.interpolant('soc_interp', 'bspline', [s], soc)
    tb_interp = cs.interpolant('tb_interp', 'bspline', [s], tb)
    tm_interp = cs.interpolant('tm_interp', 'bspline', [s], tm)
    ti_interp = cs.interpolant('ti_interp', 'bspline', [s], ti)
    tcmi_interp = cs.interpolant('tcmi_interp', 'bspline', [s], tcmi)
    tcb_interp = cs.interpolant('tcb_interp', 'bspline', [s], tcb)
    fd_interp = cs.interpolant('td_interp', 'bspline', [s], fd)
    fb_interp = cs.interpolant('tb_interp', 'bspline', [s], fb)

    x_prcom = np.zeros((N + 1, 11))
    scale = Tf/N

    for i in range(N+1):
        pos = i * scale + offset
        x_prcom[i, :] = [pos, v_interp(pos), t_interp(pos), soc_interp(pos), tb_interp(pos), tm_interp(pos),
                         ti_interp(pos), tcmi_interp(pos), tcb_interp(pos), fd_interp(pos), fb_interp(pos)]

    return x_prcom
