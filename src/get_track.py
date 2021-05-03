import numpy as np
import casadi as cs
import os
import matplotlib.pyplot as plt


def nonreg_sampling(kappa: np.ndarray,
                    eps_kappa: float = 1e-3,
                    step_non_reg: int = 0) -> tuple:
    """
    authors:
    Thomas Herrmann\n
    Maximilian Bayerlein

    .. description::
    The non-regular sampling function runs through the curvature profile and determines straight and corner sections.
    During straight sections it reduces the amount of points by skipping them depending on the step_non_reg parameter.

    .. inputs::
    :param kappa:           curvature profile (closed full track).
    :type kappa:            np.ndarray
    :param eps_kappa:       identify straights using this threshold in curvature in rad/m, i.e. straight if
                            kappa < eps_kappa
    :type eps_kappa:        float
    :param step_non_reg:    determines how many points are skipped in straight sections, e.g. step_non_reg = 3 means
                            every fourth point is used while three points are skipped
    :type step_non_reg:     int

    .. outputs::
    :return sample_idxs:    indices of points that are kept (closed full track).
    :rtype sample_idxs:     np.ndarray
    :return kappa_new:      sampled track curvatures profile (closed full track).
    :rtype track_sampled:   np.ndarray
    """

    # if stepsize is equal to zero simply return the input
    if step_non_reg == 0:
        return np.arange(0, len(kappa))

    # run through the profile to determine the indices of the points that are kept
    idx_latest = step_non_reg + 1
    sample_idxs = [0]
    kappa_new = np.array(kappa[0])

    for idx in range(1, len(kappa)):
        if np.abs(kappa[idx]) >= eps_kappa or idx >= idx_latest:
            # keep this point
            sample_idxs.append(idx)
            kappa_new = np.vstack((kappa_new, kappa[idx]))
            idx_latest = idx + step_non_reg + 1

    if len(kappa) != sample_idxs:
        sample_idxs.append(len(kappa)-1)
        kappa_new = np.vstack((kappa_new, kappa[-1]))

    return np.array(sample_idxs), np.array(kappa_new)


def get_track(laps: int,
              TRACK: dict,
              b_plot: bool = False) -> tuple:

    """
    author:
    Maximilian Bayerlein

    .. description::
    The get track function imports the curvature profile from the optimal race trajectory of a given track. The profile
    was calculated previously with the main_globaltraj function using IPOPT. Further, the curvature profile is sampled
    with a non-regular mesh.

    .. inputs::
    :param laps:            number of laps on the track.
    :type laps:             np.int
    :param TRACK:           dictionary containing data regarding the track
                            {"Name": 'Name', "discr": int, "v0": int, "Length": int}
    :type TRACK:            dict
    :param b_plot:          switch for plotting the track data
    :type b_plot:            bool

    .. outputs::
    :return s_steps:        sampled discretization horizon, containing the distance to the next discretization point
                            (closed full track). (N)
    :rtype s_steps:         np.ndarray
    :return kappa_disc:     sampled track curvature profile (closed full track).
    :rtype kappa_disc:      np.ndarray
    :return s_new:          sampled discretization points, containing the absolute position of the points (closed full
                            track). (N + 1)
    :rtype s_new:           np.ndarray
    """

    file_path = os.path.dirname(__file__)
    file_path = os.path.join(file_path, '../inputs')
    file_path = os.path.join(file_path, TRACK["Name"])

    track = np.genfromtxt(os.path.join(file_path, 'traj_race_cl.csv'), delimiter=';', skip_header=3)

    kappa = track[:, 4]
    s_ref = track[:, 0]

    kappa_temp = kappa
    s_temp = s_ref
    for i in range(laps - 1):
        # delete first entry, since it is equal to the last in a closed circuit
        # this way the number of discretization points is (len(s_ref)-1) * laps + 1
        kappa_temp = np.concatenate((kappa_temp, kappa[1:]))
        s_temp = np.concatenate((s_temp, s_ref[1:] + s_temp[-1]))

    kappa = kappa_temp
    s_ref = s_temp

    idx, kappa_new = nonreg_sampling(kappa, 0.025, 6)  # sampling of the curvature profile (full closed track)

    s_new = np.array(0)

    for i in idx:
        s_new = np.hstack((s_new, s_ref[i]))  # selection of the sampling points belonging to the sampled curvatures

    s_new = s_new[1:]  # since the array is initialized with zero, it is cut apart

    # interpolate curvature of reference line in terms of position points
    kappa_interp = cs.interpolant('kappa_interp', 'bspline', [s_ref], kappa)

    s_steps = np.zeros(len(s_new))

    # give discretization points the format of prediction horizon
    for i in range(1, len(s_new)):
        s_steps[i] = s_new[i] - s_new[i-1]

    s_steps = s_steps[1:]  # distance between discretization points (N)

    # evaluate kappa-profile at the global s coordinate with given number of discretization points along track
    kappa_disc = kappa_interp(s_new)

    # plotting a comparison of the sampled and the non-sampled curvature profile ---------------------------------------
    if b_plot:
        plt.figure()
        plt.plot(np.linspace(0, s_ref[-1], int(s_ref[-1]*2)), kappa_interp(np.linspace(0, s_ref[-1], int(s_ref[-1]*2))))
        plt.plot(s_ref, kappa, '+')
        plt.plot(s_new, kappa_new, 'm.')
        plt.legend(['kappa', 'reg', 'non_reg'])
        plt.title('Non regular kappa mesh')
        plt.show()

    return s_steps, kappa_disc, s_new
