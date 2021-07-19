import numpy as np
import os
import sys
import time

# Append custom modules to python path
mod_global_trajectory_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# print(mod_global_trajectory_path)
sys.path.append(mod_global_trajectory_path)

# Custom modules
import src
from interface import Receiver, Sender, unpack_inputs


class AcPmModel:

    def __init__(self,
                 track_name: str,
                 b_visualize: bool = False,
                 ci: bool = False):

        """Class to interface the algorithms which are necessary to calculate an energy strategy for a race vehicle.

        :param track_name: name of the track where the energy strategy shall be calculated on
        :param b_visualize: switch to visualize the energy strategy results
        :param ci: switch to be activated in the CI jobs

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.12.2020
        """

        # --------------------------------------------------------------------------------------------------------------
        # GLOBAL ATTRIBUTES --------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------
        self.CI = ci                    # switch for CI jobs
        self.track = track_name         # Track name
        self.track_data = None          # Dictionary containing the track information

        self.idx_meas = 0               # measurement idx in global s-coordinate
        self.dpx_meas_local = 0         # measurement idx in local solver indices
        self.idx_meas_last = 0          # measurement idx in global s-coordinates from previous iteration

        # Set switches -------------------------------------------------------------------------------------------------
        self.PLOT = True                # Plot results
        self.COMP_TO_IPOPT = True       # Plot IPOPT results additionally

        # Initialization of objects ------------------------------------------------------------------------------------
        self.DDref = src.DrivingDynamicsReference.DrivingDynamicsReference(b_visualize=b_visualize)
        self.ES = src.OnlineES.OnlineES(b_visualize=b_visualize)

        # Data of all callable tracks: Name, Number of discr. points, v0 [m/s], length [m] -----------------------------
        track_dict = {"Berlin": {"Name": 'Berlin', "discr": 290, "v0": 20, "Length": 2318},
                      "Hong_Kong": {"Name": 'Hong_Kong', "discr": 231, "v0": 20, "Length": 1845},
                      "Modena": {"Name": 'Modena', "discr": 251, "v0": 15, "Length": 2000},
                      "Monteblanco": {"Name": 'Monteblanco', "discr": 298, "v0": 45, "Length": 2382},
                      "Paris": {"Name": 'Paris', "discr": 235, "v0": 20, "Length": 1874},
                      "Upper_Heyford": {"Name": 'Upper_Heyford', "discr": 167, "v0": 20, "Length": 1332}}

        # Select track according to args -------------------------------------------------------------------------------
        self.track_data = track_dict[self.track]

        print('[INFO] ES set up for ' + track_name + '\n')

    # ------------------------------------------------------------------------------------------------------------------
    # - DEFINE create_dd_ref METHOD ------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    def create_dd_ref(self):
        """Calls the calculation of the reference velocity profile.

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.12.2020
        """

        stat = self.DDref.create_ref(track_data=self.track_data)
        if self.CI:
            if stat == 0:
                sys.exit(0)
            else:
                sys.exit(1)

    # ------------------------------------------------------------------------------------------------------------------
    # - DEFINE init_es METHOD ------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    def init_es(self,
                x0: np.array,
                num_laps: int):
        """Calls the initialization of the energy strategy.

        :param x0: initial values of the DAEs
        :param num_laps: number of race laps

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.12.2020
        """

        # Overwrite velocity of initial state with a feasible one ------------------------------------------------------
        # s_pm, v, t_pm, soc_batt, temp_batt, temp_mach, temp_inv, temp_cool_mi, temp_cool_b
        x0[1] = self.track_data["v0"]

        stat = self.ES.initialize_es(x0=x0,
                                     track_data=self.track_data,
                                     laps=num_laps)

        if self.CI:
            if stat == 0:
                sys.exit(0)
            else:
                sys.exit(1)

    # ------------------------------------------------------------------------------------------------------------------
    # - DEFINE re-optimization METHOD ----------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    def re_optimize(self,
                    dp: int,
                    dp_dist: int,
                    x_meas: np.array):
        """Calls the reoptimization of the energy strategy.

        :param dp: bounds on dp_th node
        :param dp_dist: local position index of travelled vehicle in current solver instance
        :param x_meas: measurement value

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.12.2020
        """

        # bind measurement values on the states to max. and min. constraints
        x_meas = es.ES.bind_meas_to_constraints(x_meas=x_meas)

        self.ES.recalc_es(dpx=dp,
                          dpx_dist=dp_dist,
                          x0=x_meas)

    # ------------------------------------------------------------------------------------------------------------------
    # - DEFINE get_idx METHOD ------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    def get_idx(self,
                vp_in: dict):
        """Calls the reoptimization of the energy strategy.

        :param vp_in: velocity planner input

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.03.2021
        """

        # get discrete position in NLP matching the s-coordinate of the measurement
        self.idx_meas = (np.abs(self.ES.s_track - vp_in['s_meas'])).argmin()
        print("[INFO] Triggered ES re-optimization on s-ID (new x0): ", self.idx_meas)

        # measurement ID in local solver
        self.dpx_meas_local = self.idx_meas - self.idx_meas_last

        # overwrite last measurement ID with current one for next iteration
        self.idx_meas_last = self.idx_meas


def translate_track_id(track_id: str) -> str:
    """Translates the track ID into its full name.

    :param track_id: short ID of race track

    :return track_name: full name of race track

    :Authors:
        Thomas Herrmann <thomas.herrmann@tum.de>

    :Created on:
        01.12.2020
    """

    if track_id == 'mnt':
        track_name = 'Monteblanco'
    elif track_id == 'mod':
        track_name = 'Modena'
    elif track_id == 'prs':
        track_name = 'Paris'
    elif track_id == 'ber':
        track_name = 'Berlin'
    elif track_id == 'hok':
        track_name = 'Hong_Kong'
    elif track_id == 'upp':
        track_name = 'Upper_Heyford'
    else:
        print('No valid track ID found. Exiting.')
        sys.exit(1)

    return track_name


if __name__ == "__main__":
    """This function provides an example on how to use the energy strategy algorithm in this repository.

    :Authors:
        Thomas Herrmann <thomas.herrmann@tum.de>

    :Created on:
        01.12.2020
    """

    # --- frequency of main loop task
    f_main_loop_Hz = 1
    b_visual = False

    # --- Check command line args --------------------------------------------------------------------------------------
    if len(sys.argv) >= 2:
        # activate CI behavior
        b_ci = bool(int(sys.argv[1]))
    else:
        b_ci = False

    # --- initialize interfaces ----------------------------------------------------------------------------------------
    zmq_rcv = Receiver.ZMQReceiver(theme="sender_imp_esim")
    zmq_snd = Sender.ZMQSender(theme='receiver_exp_vplanner')

    # --- embedded ES main loop ----------------------------------------------------------------------------------------
    while True:
        print('ES online.')

        # --- get input from velocity planner module
        vp_input = zmq_rcv.run()
        if vp_input is not None:
            unpack_inputs.unpack(msg_in=vp_input)

            # --- Case: Calculate v_ref
            if vp_input['phase'] == 'v':
                track_name_ = translate_track_id(vp_input['track'])

                es = AcPmModel(track_name=track_name_,
                               b_visualize=b_visual,
                               ci=b_ci)

                es.create_dd_ref()
                # delete solver object as a new solver is necessary for all future online functionality
                del es

            # --- Case: Initialize ES
            elif vp_input['phase'] == 'i':
                track_name_ = translate_track_id(vp_input['track'])

                es = AcPmModel(track_name=track_name_,
                               b_visualize=b_visual,
                               ci=b_ci)

                es.init_es(x0=vp_input['x0'],
                           num_laps=vp_input['num_laps'])

            # --- Case: Re-optimize ES
            elif vp_input['phase'] == 'r':

                # get discrete position in NLP matching the s-coordinate of the measurement
                es.get_idx(vp_in=vp_input)

                # measurement value: calculated state + measurement difference
                x_meas_ = \
                    es.ES.res.x[es.dpx_meas_local, :] \
                    + vp_input['meas_diff']

                # re-create solver with less discretization points
                es.re_optimize(dp=es.idx_meas,
                               dp_dist=es.dpx_meas_local,
                               x_meas=x_meas_)

                print("[INFO] New state at measurement point: ", es.ES.res.x[0, :])

            # --- Case: not known
            else:
                print('Specified ES phase not known! Exiting.')
                sys.exit(1)

            # --- send variable power limits to trajectory planning module
            if vp_input['phase'] == 'i' or vp_input['phase'] == 'r':

                # data: global s (without last coordinate) and max. power values in between discretization points
                s_glo = es.ES.res.x[:-1, 0] + es.ES.s_track[es.idx_meas]
                zmq_snd.send(data=np.column_stack((s_glo, es.ES.res.pwr)))

        time.sleep(1 / f_main_loop_Hz)
