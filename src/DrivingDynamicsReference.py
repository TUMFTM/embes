import numpy as np
from matplotlib import pyplot as plt
# custom modules
import src


class DrivingDynamicsReference:

    def __init__(self,
                 b_visualize: bool = False):
        """Class to calculate the reference velocity profile under full-speed operation.

        :param b_visualize: turn on plotting

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.10.2020
        """

        self.x0 = None
        self.s_track = None
        self.s_ref = None
        self.kappa_disc = None
        self.b_visualize = b_visualize

        self.res = src.GetESresults.ESres()

    def create_ref(self,
                   track_data: dict) -> int:
        """Calculates the reference velocity profile.

        :param track_data: dictionary containing track length and discretization steps

        :return int: return 0 if everything is OK

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.10.2020
        """

        # --------------------------------------------------------------------------------------------------------------
        # - DEFINE TRACK FOR DRIVING DYNAMICS REFERENCE ----------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------
        # --- Import SINGLE race lap with variable mesh
        [self.s_ref, self.kappa_disc, self.s_track] = src.get_track.get_track(laps=1,
                                                                              TRACK=track_data,
                                                                              b_plot=self.b_visualize)

        # double delta_s for two race laps (start + flying lap)
        self.s_ref = np.tile(self.s_ref, 2)
        # double kappa_disc by removing first entry in kappa array and appending to it
        self.kappa_disc = np.concatenate((self.kappa_disc, self.kappa_disc[1:]))

        # calculate global s coordinate starting from 0
        s_global = np.insert(np.cumsum(self.s_ref), 0, 0)

        # Get number of discretization points
        N = len(self.s_ref)

        # plot input track data
        if self.b_visualize:
            plt.figure()
            plt.plot(s_global, self.kappa_disc, '-*')
            plt.xlabel('Global s in m')
            plt.ylabel('Curvature kappa in rad/m')
            plt.title('Driving Dynamics reference laps curvature')
            plt.show()

        # define system ODEs and constraints to create driving dynamics reference speed profile,
        # the loss model choice is not of interest here, as the thermodynamics and power losses do not influence the
        # result but the calculation time
        es_model, es_ddref_constr = src.def_es_model.es_model(b_dd_ref=True,
                                                              b_battery_simple=True,
                                                              b_machine_simple=False,
                                                              b_inverter_simple=False)

        # Set x0 to avoid reaching a thermodynamic constraint during optimization of one single lap
        # s, v, t, SOC_batt, temp_batt, temp_mach, temp_inv, temp_cool_mi, temp_cool_b
        self.x0 = np.array([0, track_data["v0"], 0, 0.9, 30, 30, 30, 30, 30])

        # --------------------------------------------------------------------------------------------------------------
        # - DEFINE SOLVER SETTINGS FOR DRIVING DYNAMICS REFERENCE ------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------
        acados_solver = src.def_es_solver.es_solver(model=es_model,
                                                    constraint=es_ddref_constr,
                                                    time_steps=self.s_ref,
                                                    hessian_method="EXACT",
                                                    x0=self.x0,
                                                    b_dd_ref=True)

        for i in range(0, N + 1):
            # insert the discrete track kappa values
            acados_solver.set(i, "p", np.array([self.kappa_disc[i]]))
            # insert initial solver guess for the state variables
            acados_solver.set(i, 'x', np.hstack(self.x0))

        # --------------------------------------------------------------------------------------------------------------
        # - SOLVE ------------------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------
        print('[INFO] Solving to create driving dynamics reference speed profile ...')

        status = acados_solver.solve()
        print("[INFO] DD reference | solving time [s]:", float(acados_solver.get_stats('time_tot')))
        print("[INFO] DD reference | SQP iterations:", int(acados_solver.get_stats('sqp_iter')))
        if status != 0:
            print("[ERROR] Problem could not be solved. Exiting.")
            return 1

        # reference velocity [m/s]
        dd_ref = np.zeros((N + 1, es_model.x.size()[0]))

        # controls [kN]
        f_arr = np.zeros((N, es_model.u.size()[0]))
        # power [kW]
        p_arr = np.zeros((N, 1))
        # combined acceleration
        tre_arr = np.zeros((N, 1))

        for i in range(N):
            dd_ref[i, :] = acados_solver.get(i, 'x')
            f_arr[i, :] = acados_solver.get(i, 'u')
            p_arr[i, :] = es_ddref_constr.p_drive(acados_solver.get(i, 'x'), acados_solver.get(i, 'u'))
            tre_arr[i, :] = es_ddref_constr.kamm_c(acados_solver.get(i, 'x'), acados_solver.get(i, 'u'),
                                                   self.kappa_disc[i])
        dd_ref[N, :] = acados_solver.get(N, 'x')

        if self.b_visualize:
            plt.figure()
            plt.plot(dd_ref[int(N / 2):, 0], dd_ref[int(N / 2):, 1])
            plt.plot(dd_ref[int(N / 2):-1, 0], f_arr[int(N / 2):, 0])
            plt.plot(dd_ref[int(N / 2):-1, 0], f_arr[int(N / 2):, 1])
            plt.plot(dd_ref[int(N / 2):-1, 0], 0.2 * p_arr[int(N / 2):, 0])
            plt.plot(dd_ref[int(N / 2):-1, 0], 0.1 * tre_arr[int(N / 2):, 0])
            plt.title('Driving dynamics reference speed profile')

        self.res.convert_results(es_model=es_model,
                                 es_constraints=es_ddref_constr,
                                 es_solver=acados_solver,
                                 kappa_disc=self.kappa_disc)

        if self.b_visualize:

            self.res.show_statistics(solver=acados_solver)

            self.res.plot_results(es_model=es_model)

            plt.show()

        # use the second flying lap as driving dynamics reference with maximum floating point precision
        np.savetxt('v_ref.csv', np.column_stack((dd_ref[int(N / 2):, 1])), '%.32f')
        np.savetxt('T_mach_ref.csv', np.column_stack((dd_ref[:int(N / 2) + 1, 5])), '%.32f')

        print('[INFO] Driving dynamics reference speed profile successfully created.')

        return 0


if __name__ == '__main__':
    pass
