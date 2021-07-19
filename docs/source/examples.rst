.. _refExamples:

********
Examples
********

In the folder `examples/`, we provide the file `use_emb_es.py`, which opens a ZMQ Sender and Receiver to communicate with the
energy strategy algorithm. In the following, we wish to explain the code fragments in its **main**.

**Important:** :ref:`refStep1` must be executed prior to the race start. :ref:`refStep2` must be run to initialize
the energy
strategy.
:ref:`refStep3` is running online to recalculate the strategy.

.. _refStep1:

1. Create Reference velocity profile
------------------------------------
Creation of the reference velocity profile `v_ref` is done by

.. code-block:: python

    vpl.trigger_recalc(phase='v',
                       track='mnt')

By this, the reference velocity profile is generated and saved as a `.csv`-file.

.. _refStep2:

2. Initialization of the energy strategy
----------------------------------------
Initialization of the energy strategy is done by

.. code-block:: python

    # x0 = s [m], v[m/s], t[s], soc_batt [], temp_batt, temp_mach, temp_inv, temp_cool_mi, temp_cool_b [°C]
    vpl.trigger_recalc(s_meas=0.0,
                       meas_diff=np.array(0),
                       phase='i',
                       track='mnt',
                       laps=12,
                       x0=np.array([1, 0, 0, 0.5, 35, 35, 35, 35, 35]))

Here, `s_meas` indicates the global s-coordinate where a powertrain measurement of SOC and temperatures was made
(start: `s_meas` = 0.0m). The number of race laps must be set in the argument `laps` for a specific `track`. The initial state conditions (see code comment above)
are provided in `x0`. Have a look in :ref:`availableTracks` to find available race tracks and their respective
IDs.

.. _refStep3:

3. Reoptimization of the energy strategy
-----------------------------------------
Re-optimization of the energy strategy is done by

.. code-block:: python

    s_meas_ = 500  # meters
    # meas_diff = s [m], v[m/s], t[s], soc_batt [], temp_batt, temp_mach, temp_inv, temp_cool_mi, temp_cool_b [°C]
    vpl.trigger_recalc(s_meas=s_meas_,
                       meas_diff=np.array([0, 0, 0, -0.02, 0, 3.0, 0, 0, 0]))

Here, `s_meas` describes the global s-coordinate where the measurement value was generated. The measurement differences
compared to the planned values of the last energy strategy recalculation at this position need to be set in `meas_diff`.

.. _OpenLoop:

Open loop simulation (internal)
-------------------------------

The energy strategy can be run in an open loop mode. This example explains how to run

    - a trajectory planner,
    - the `velocity optimization module <https://github.com/TUMFTM/velocity_optimization>`_,
    - an LQR vehicle dynamics controller,
    - a vehicle dynamics simulation,
    - the energy strategy.

To do this, a trajectory planner, which uses the
`velocity_optimization <https://github.com/TUMFTM/velocity_optimization>`_ module must be started. It transfers local trajectories to the LQR controller, and the vehicle
dynamics simulation. The trajectory planner itself calls the initialization :ref:`refStep2` and the reoptimization
:ref:`refStep3` of the energy strategy module. Do the following steps to setup the open loop simulation:

    1. Clone the repository `mod_control <https://gitlab.lrz.de/iac/mod_control>`_ and checkout branch `old_rr_ltpl`:

        .. code-block:: bash

            git clone https://gitlab.lrz.de/iac/mod_control
            git checkout old_rr_ltpl

    2. Download the latest controller and vehicle dynamics simulation from `branch old_rr_ltpl <https://gitlab.lrz
    .de/iac/mod_control/-/tree/old_rr_ltpl>`_ and extract the files into `/mod_control/misc/py_binds/dist/` and install via

        .. code-block:: bash

            pip install tum_motion_control-0.1.0-cp38-cp38-linux_x86_64

    3. Install rticonnextdds-connector in a virtual environment with Python 3.8 via

        .. code-block:: bash

            pip install rticonnextdds-connector

    4. Run the main script of the controller simulation via

        .. code-block:: bash

            python3 raceline_driving.py --vehicle_interface sil --vehicle_params il --network_config loc --slow_factor 1.0 --verbose False

    5. Go ahead, start the energy strategy and the trajectory planner.
