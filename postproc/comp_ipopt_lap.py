from matplotlib import pyplot as plt
import numpy as np
import tikzplotlib
from scipy import interpolate


def comp_ipopt_lap(s_global: np.array,
                   v_ref_long: np.array):
    """Comparison script for a single race lap to IPOPT ZSM setup (2 laps for straight comparison to HPIPM as we do not
    know the starting velocity in the HPIPM solver in the first lap). The velocity pforiles and the machine
    temperature profiles will be compared.
    Note: v_ref.csv must be created with v0 set to the correct velocity of the flying lap. Otherwise, the temperature
    profile calculated by HPIPM is not exactly correct. The used data files are produced by the DrivingDynamicsReference
    class and by the IPOPT ZSM setup using non-regular sampling.
    Provide the correct paths to your data files below!

    :param s_global: array containing the global s-coordinate in [m] of the comparison data.
    :param v_ref_long: reference vleocity profile (v_ref multiplied by the given number of race laps).

    :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

    :Created on:
        01.02.2021
    """

    # --- User specific inputs files
    # file containing states from IPOPT setup optimization for single race lap
    f_states_ipopt = 'postproc/example/states_ipopt_nonreg.csv'
    # reference velocity profile by HPIPM for single race lap using DrivingDynamicsReference
    f_v_ref = 'v_ref.csv'
    # reference machine temperature profile by HPIPM for a single race lap using DrivingDynamicsReference
    f_T_mach_ref = 'postproc/example/T_mach_ref.csv'

    # --- save DD reference over entire race ---------------------------------------------------------------------------
    plt.plot(s_global, v_ref_long)

    tikzplotlib.save('v_ref_long.tex')

    # --- velocity comparison for single lap to IPOPT ------------------------------------------------------------------
    r_ipopt_zsm = np.loadtxt(f_states_ipopt, usecols=[0, 2, 7], delimiter=';')
    r_hpipm_pm = np.loadtxt(f_v_ref)
    plt.figure()
    plt.plot(s_global[:r_hpipm_pm.size], r_hpipm_pm)
    plt.plot(r_ipopt_zsm[:, 0], r_ipopt_zsm[:, 1])
    plt.xlabel('s in m'); plt.ylabel('v in mps'); plt.legend(['PM HPIPM', 'ZSM IPOPT'])

    tikzplotlib.save('v_hpipm_ipopt.tex')

    # --- calc NRMSE for v between IPOPT ZSM and HPIPM PM
    # create interpolant to compare both optim. results
    r_ipopt_zsm_interp = interpolate.interp1d(r_ipopt_zsm[:, 0], r_ipopt_zsm[:, 1],
                                              bounds_error=False, fill_value='extrapolate')

    print('NRMSE for velocity IPOPT ZSM vs HPIPM PM [%]',
          np.sqrt(np.sum((r_ipopt_zsm_interp(s_global[:r_hpipm_pm.size]) - r_hpipm_pm) ** 2) /
          r_hpipm_pm.size) / (np.max(r_ipopt_zsm[:, 1]) - np.min(r_ipopt_zsm[:, 1])) * 100)

    # --- temperature comparison for single lap to IPOPT ---------------------------------------------------------------
    t_hpipm_pm = np.loadtxt(f_T_mach_ref)
    plt.figure()
    plt.plot(s_global[:t_hpipm_pm.size], t_hpipm_pm)
    plt.plot(r_ipopt_zsm[:, 0], r_ipopt_zsm[:, 2])
    plt.xlabel('s in m'); plt.ylabel('T_M in °C'); plt.legend(['PM HPIPM', 'ZSM IPOPT'])

    t_ipopt_zsm_interp = interpolate.interp1d(r_ipopt_zsm[:, 0], r_ipopt_zsm[:, 2],
                                              bounds_error=False, fill_value='extrapolate')

    print('NRMSE for temperature profile IPOPT ZSM vs HPIPM PM [%]',
          np.sqrt(np.sum((t_ipopt_zsm_interp(s_global[:t_hpipm_pm.size]) - t_hpipm_pm) ** 2) /
                  t_hpipm_pm.size) / (np.max(r_ipopt_zsm[:, 2]) - np.min(r_ipopt_zsm[:, 2])) * 100)

    tikzplotlib.save('T_hpipm_ipopt.tex')

    plt.show()
