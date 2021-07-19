import sys
import numpy as np
import time
# custom modules
import src
from postproc import comp_ipopt_lap


class OnlineES:

    def __init__(self,
                 b_visualize: bool = False):
        """Class containing all the functionality to initialize and recalculate the energy strategy.

        :param b_visualize: activate interactive plots after every optimization of the energy strategy.

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.12.2020
        """

        self.s_ref = None
        self.kappa_disc = None
        self.s_track = None
        self.v_ref = None
        self.es_model = None
        self.es_constraints = None
        self.es_solver = None
        self.nlp_steplen_init = None

        self.b_visualize = b_visualize

        self.res = src.GetESresults.ESres()

    def initialize_es(self,
                      x0: np.array,
                      track_data: dict,
                      laps: int) -> int:
        """Initializes the energy strategy. This is the first attempt to create an energy for a given race lap,
        which takes a reference velocity profile as an initial guess. You can create this reference velocity profile
        using the class `DrivingDynamicsReference`.

        :param x0: initial values of the ODEs
        :param track_data: dictionary containing all the necessary track information
        :param laps: number of race laps

        :return stat: return the solver status, which must be '0' for success

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.12.2020
        """

        b_save_tikz = self.b_visualize

        # load driving dynamics reference speed profile
        self.v_ref = np.loadtxt('v_ref.csv', dtype=np.float64)

        # --------------------------------------------------------------------------------------------------------------
        # - GET TRACK DATA ---------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------
        # import SINGLE race lap with variable mesh
        [self.s_ref, self.kappa_disc, self.s_track] = src.get_track.get_track(laps=1,
                                                                              TRACK=track_data)
        # multiples of delta_s
        self.s_ref = np.tile(self.s_ref, laps)
        # multiples of kappa_disc by removing first entry in kappa array and appending to it
        tmp_kappa_disc = self.kappa_disc
        tmp_v_ref = self.v_ref
        for i in range(0, laps - 1):
            self.kappa_disc = np.concatenate((self.kappa_disc, tmp_kappa_disc[1:]))
            self.v_ref = np.concatenate((self.v_ref, tmp_v_ref[1:]))

        # calculate global s coordinate starting from 0
        s_global = np.insert(np.cumsum(self.s_ref), 0, 0)
        self.s_track = s_global

        # Get number of discretization points
        N = len(self.s_ref)

        # --------------------------------------------------------------------------------------------------------------
        # SET MODEL AND SOLVER -----------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        # initialize model and constraints
        self.es_model, self.es_constraints = src.def_es_model.es_model(b_dd_ref=False,
                                                                       b_battery_simple=False,
                                                                       b_inverter_simple=False,
                                                                       b_machine_simple=False)

        # set solver and its costs -------------------------------------------------------------------------------------
        self.es_solver = src.def_es_solver.es_solver(model=self.es_model,
                                                     constraint=self.es_constraints,
                                                     time_steps=self.s_ref,
                                                     hessian_method="EXACT",
                                                     x0=x0)

        if b_save_tikz:
            comp_ipopt_lap.comp_ipopt_lap(s_global=s_global,
                                          v_ref_long=self.v_ref)

        for i in range(0, N + 1):
            # set the track kappa values
            self.es_solver.set(i, "p", np.array([self.kappa_disc[i]]))
            # --- set the initial guess for the state variables using v_ref
            self.es_solver.set(i, 'x', np.hstack((x0[0], 0.5 * self.v_ref[i], x0[2:])))
            if i < N:
                self.es_solver.set(i, 'u', np.array([0.5 * self.es_model.f_drive_max, 0]))

        # sets x in acados to x0-constraint to reduce feasibility issues
        self.set_state_constraints(x=x0)

        # --- bounds on first node: set x0 manually --------------------------------------------------------------------

        print('[INFO] Initializing ES ...')
        self.nlp_steplen_init, \
            stat = \
            self.solve_es(N=N,
                          kappa_disc_recalc=self.kappa_disc,
                          b_save_tikz=b_save_tikz)

        return stat

    def recalc_es(self,
                  dpx: int,
                  dpx_dist: int,
                  x0: np.array):
        """Recalculates the energy strategy. Based on current measurement data (x0), a reoptimization will take place
        for the remaining race distance.

        :param x0: initial values of the ODEs (measurement)
        :param dpx: index in the global s-coordinate where the vehicle currently is and the measurements were taken
        :param dpx_dist: local index in the global s-coordinate where the vehicle currently is and the measurements
        were taken

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.12.2020
        """

        b_save_tikz = self.b_visualize

        # --------------------------------------------------------------------------------------------------------------
        # - GET REDUCED TRACK DATA -------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------
        ts = time.time()

        # Get number of discretization points
        s_ref_recalc = self.s_ref[dpx:]
        kappa_disc_recalc = self.kappa_disc[dpx:]  # length: N + 1
        N = len(s_ref_recalc)

        # --------------------------------------------------------------------------------------------------------------
        # - SET NEW SOLVER ---------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        # set solver and its costs -------------------------------------------------------------------------------------
        # delete old solver instance to free bound libraries
        del self.es_solver
        # this solver has less discretization points starting from the current position of the vehicle
        # TODO: try different initial NLP steplength from initialization solver
        self.es_solver = src.def_es_solver.es_solver(model=self.es_model,
                                                     constraint=self.es_constraints,
                                                     time_steps=s_ref_recalc,
                                                     hessian_method="EXACT",
                                                     x0=x0,
                                                     nlp_steplen=self.nlp_steplen_init)

        # --- set ALL the available values from the previous solving process to speed up the computation ---------------
        for i in range(0, N + 1):
            # set the track kappa values for the reduced track
            self.es_solver.set(i, "p", np.array([kappa_disc_recalc[i]]))

            # states x
            self.es_solver.set(i, 'x', self.res.x[i + dpx_dist, :])

            # controls u
            if i < N:
                u_guess = self.res.u[i + dpx_dist, :]
                # to enforce the complementarity constraint F_d * F_b == 0
                if u_guess[0] > 0.5:
                    u_guess[1] = 0
                elif u_guess[1] < -0.5:
                    u_guess[0] = 0
                self.es_solver.set(i, 'u', u_guess)

        self.set_state_constraints(x=x0)

        print('[INFO] Resolving with measurement value ...')
        self.solve_es(N=N,
                      kappa_disc_recalc=kappa_disc_recalc,
                      b_save_tikz=b_save_tikz,
                      dpx=dpx)

        print("[INFO] Re-solving time [s]:", time.time() - ts)

    def set_state_constraints(self,
                              x: np.array):
        """Set solver values to match the set constraints on the initial DAE value

        :param x: states vector on first discretization point

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.12.2020
        """

        # set solver values to match the set constraints on the initial DAE value
        self.es_solver.set(0, 'x', x)

    def reset_state_constraints(self):

        # --- Reset of constraints on specific discretization point (idx: 0)

        """
        # lower bounds
        self.es_solver.\
            set(0, 'lbx',
                np.array((self.es_model.v_min, self.es_model.soc_min, self.es_model.temp_batt_min,
                          self.es_model.temp_mach_min, self.es_model.temp_inv_min,
                          self.es_model.temp_cool_mach_inv_min, self.es_model.temp_cool_batt_min)))
        # upper bounds
        self.es_solver.\
            set(0, 'ubx',
                np.array((self.es_model.v_max, self.es_model.soc_max, self.es_model.temp_batt_max,
                          self.es_model.temp_mach_max, self.es_model.temp_inv_max,
                          self.es_model.temp_cool_mach_inv_max, self.es_model.temp_cool_batt_max)))
        """
        pass

    def bind_meas_to_constraints(self,
                                 x_meas: np.array) -> np.array:
        """Binds the measurement values to the linear OCP state constraints.

        :param x_meas: measured states vector

        :return x_meas: returns modified measurement values

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.12.2020
        """

        # Velocity [m/s]
        id_state = 1
        if x_meas[id_state] > self.es_model.v_max: x_meas[id_state] = self.es_model.v_max
        if x_meas[id_state] < self.es_model.v_min: x_meas[id_state] = self.es_model.v_min
        # SOC []
        id_state = 3
        if x_meas[id_state] > self.es_model.soc_max: x_meas[id_state] = self.es_model.soc_max
        if x_meas[id_state] < self.es_model.soc_min: x_meas[id_state] = self.es_model.soc_min
        # T_Batt [degC]
        id_state = 4
        if x_meas[id_state] > self.es_model.temp_batt_max: x_meas[id_state] = self.es_model.temp_batt_max
        if x_meas[id_state] < self.es_model.temp_batt_min: x_meas[id_state] = self.es_model.temp_batt_min
        # T_Machine [degC]
        id_state = 5
        if x_meas[id_state] > self.es_model.temp_mach_max: x_meas[id_state] = self.es_model.temp_mach_max
        if x_meas[id_state] < self.es_model.temp_mach_min: x_meas[id_state] = self.es_model.temp_mach_min
        # T_Inverter [degC]
        id_state = 6
        if x_meas[id_state] > self.es_model.temp_inv_max: x_meas[id_state] = self.es_model.temp_inv_max
        if x_meas[id_state] < self.es_model.temp_inv_min: x_meas[id_state] = self.es_model.temp_inv_min
        # T_Cool_MI [degC]
        id_state = 7
        if x_meas[id_state] > self.es_model.temp_cool_mach_inv_max:
            x_meas[id_state] = self.es_model.temp_cool_mach_inv_max
        if x_meas[id_state] < self.es_model.temp_cool_mach_inv_min:
            x_meas[id_state] = self.es_model.temp_cool_mach_inv_min
        # T_Cool_B [degC]
        id_state = 8
        if x_meas[id_state] > self.es_model.temp_cool_batt_max: x_meas[id_state] = self.es_model.temp_cool_batt_max
        if x_meas[id_state] < self.es_model.temp_cool_batt_min: x_meas[id_state] = self.es_model.temp_cool_batt_min

        return x_meas

    def solve_es(self,
                 N: int,
                 kappa_disc_recalc: np.array,
                 b_save_tikz: bool,
                 dpx: int = 0):
        """Solves the OCP to provide energy strategy results.

        :param N: Length of discrete optimization horizon
        :param kappa_disc_recalc: curvature of the track, where the ES shall be solved on [rad/m]
        :param b_save_tikz: bool to save results as separate .tex-files
        :param dpx: discrete position index in the global s-coordinate frame where the vehicle currently is

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.12.2020
        """

        print("[INFO] NLP steplength:", self.es_solver.acados_ocp.solver_options.nlp_solver_step_length)

        # --- solve the given problem
        status = self.es_solver.solve()

        # --- First check: QP solver returned failure
        if status == 4:
            for i in range(0, N + 1):
                if i < N:  # control guess u based on scenario. Longer run --> lower u-guess
                    self.es_solver.set(i, 'u', np.array([0.1 * self.es_model.f_drive_max, 0]))

            # resolve OCP
            print("[INFO] ES re-solving with reduced control inputs guess ...")
            status = self.es_solver.solve()

            if status == 0:
                print("[INFO] ES re-solve | solving time [s]:", float(self.es_solver.get_stats('time_tot')))
                print("[INFO] ES re-solve | SQP iterations:", int(self.es_solver.get_stats('sqp_iter')))

                return 0
            else:
                print("[ERROR] ES re-solve | Can't get rid of solver error! Check your problem.")

        nlp_steplength = 1.0
        # --- Second check: Problem was solved directly
        if status == 0:
            print("[INFO] ES solver successful.")
            print("[INFO] ES | NLP solver status:", status)
            print("[INFO] ES | solving time [s]:", float(self.es_solver.get_stats('time_tot')))
            print("[INFO] ES | SQP iterations:", int(self.es_solver.get_stats('sqp_iter')))

        # --- Third check: NLP iterations limit
        elif status == 2:

            nlp_steplength_reduction = 0.25
            nlp_steplength = self.es_solver.acados_ocp.solver_options.nlp_solver_step_length

            for i in range(1, int(1 / nlp_steplength_reduction)):

                print("[INFO] ES | solving time [s]:", float(self.es_solver.get_stats('time_tot')))
                print("[INFO] ES problem seems to be challenging. Reducing NLP step length.")

                if self.es_solver.acados_ocp.solver_options.nlp_solver_step_length - nlp_steplength_reduction > 0.0:

                    # reduce current NLP solver step length
                    nlp_steplength -= nlp_steplength_reduction
                    # set new step length in OCP solver
                    self.es_solver.options_set('step_length', nlp_steplength)
                    print("[INFO] NLP step length reduced to", nlp_steplength)

                    # resolve OCP
                    print("[INFO] Solving again to optimize ES ...")
                    status = self.es_solver.solve()
                    if status == 0:
                        print("[INFO] ES | solving time [s]:", float(self.es_solver.get_stats('time_tot')))
                        print("[INFO] ES | SQP iterations:", int(self.es_solver.get_stats('sqp_iter')))
                        print("[INFO] ES solver successful.")
                        break
                    else:
                        print("[INFO] ES | solving time [s]:", float(self.es_solver.get_stats('time_tot')))
                else:
                    print("[ERROR] ES problem solver failed with reduced NLP step length.")
                    sys.exit(1)

        # --- Last check: Unknown status
        else:
            print("[ERROR] ES optimization failed.")

        self.res.convert_results(es_model=self.es_model,
                                 es_constraints=self.es_constraints,
                                 es_solver=self.es_solver,
                                 kappa_disc=kappa_disc_recalc)

        if self.b_visualize:

            # self.res.show_statistics(solver=self.es_solver)

            self.res.plot_results(es_model=self.es_model,
                                  s_glo_offset=self.s_track[dpx],
                                  b_save_tikz=b_save_tikz)

        return nlp_steplength, status


if __name__ == '__main__':
    pass
