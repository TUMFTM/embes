import numpy as np
import time
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from interface import Sender

if __name__ == "__main__":
    """Function to check the initialization phase of the energy strategy on the Berlin circuit."""

    zs = Sender.ZMQSender(theme='receiver_exp_esim')

    # test interface: start ESIM and v-planner afterwards: a message on the ESIM-side should appear.
    # --- v_ref: 'v' + track-ID
    phase = 'i'
    track = 'ber'
    # --- init.: 'i' + track-ID + number of laps + initial state x0
    laps = 25
    # s [m], v[m/s], t[s], soc_batt [], temp_batt, temp_mach, temp_inv, temp_cool_mi, temp_cool_b [°C]
    x0 = np.array([1, 0, 0, 1.0, 35, 35, 35, 35, 35])
    # --- re-optim.: 'r' + global coordinate of measurement diff. [m] + measurement diff. [various]
    s_meas = 70
    # s [m], v[m/s], t[s], soc_batt [], temp_batt, temp_mach, temp_inv, temp_cool_mi, temp_cool_b [°C]
    meas_diff = np.array([0, 0, 0, 0, 0.5, 30, 0, 0, 0])

    # --- pack data
    zs_data = dict()
    zs_data['phase'] = phase
    zs_data['track'] = track
    zs_data['num_laps'] = laps
    zs_data['x0'] = x0
    zs_data['s_meas'] = s_meas
    zs_data['meas_diff'] = meas_diff

    # --- wait for ES to go online
    time.sleep(10)
    zs.send(zs_data)
