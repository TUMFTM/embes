import types
import matplotlib.pyplot as plt
import numpy as np
import tikzplotlib
from acados_template import AcadosOcpSolver

X_PLOTS_THERM = 3
Y_PLOTS_THERM = 2
X_PLOTS_DYN = 3
Y_PLOTS_DYN = 1

V_MACH_LOSS = 2
H_MACH_LOSS = 0
V_INV_LOSS = 1
H_INV_LOSS = 0
V_BAT_LOSS = 0
H_BAT_LOSS = 0

V_BAT_TEMP = 0
H_BAT_TEMP = 1
V_INM_TEMP = 1
H_INM_TEMP = 1

V_VDC = 2
H_VDC = 1

V_VEL = 0
H_VEL = 0
V_FOR = 1
H_FOR = 0
V_TRE = 2


class ESres:

    def __init__(self):
        """Class to extract and store the optimization results from acados.

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.10.2020
        """

        self.XPLOTS = 3
        self.YPLOTS = 3
        pass

    def convert_results(self,
                        es_model: types.SimpleNamespace,
                        es_constraints: types.SimpleNamespace,
                        es_solver: AcadosOcpSolver,
                        kappa_disc: np.array):
        """Converts the results from the AcadosOcpSolver to interpretable values.

        :param es_model: optimization model (CasADi language)
        :param es_constraints: optimization constraints (CasADi language)
        :param es_solver: acados solver object
        :param kappa_disc: discrete track curvature values [rad/m]

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.10.2020
        """

        # dimensions
        N = es_solver.acados_ocp.dims.N

        # states
        x_arr = np.zeros((N + 1, es_model.x.size()[0]))
        # controls
        u_arr = np.zeros((N, es_model.u.size()[0]))
        # algebraic variable
        z_arr = np.zeros((N, es_model.z.size()[0]))
        # lower, upper slacks
        sl_arr, su_arr = np.zeros((N, len(es_solver.acados_ocp.cost.zu))), \
                         np.zeros((N, len(es_solver.acados_ocp.cost.zu)))
        # combined acceleration, power constraints
        tre_arr, p_arr, p_total_arr = np.zeros((N, 1)), np.zeros((N, 1)), np.zeros((N, 1))
        # lateral, longitudinal force
        f_late_arr, f_long_arr = np.zeros((N + 1, 1)), np.zeros((N, 1))

        # Cooling temperatures
        temp_cool_12_arr, temp_cool_13_arr = np.zeros((N + 1, 1)), np.zeros((N + 1, 1))

        # Component losses
        bat_loss = np.zeros((N, 1))
        v_dc, Ri = np.zeros((N + 1, 1)), np.zeros((N + 1, 1))
        machine_loss, \
            machine_loss_copper,\
            machine_loss_statoriron,\
            machine_loss_rotor = np.zeros((N, 1)), np.zeros((N, 1)), np.zeros((N, 1)), np.zeros((N, 1))
        inverter_loss, \
            inverter_loss_switching, \
            inverter_loss_conducting = np.zeros((N, 1)), np.zeros((N, 1)), np.zeros((N, 1))

        # Fill arrays --------------------------------------------------------------------------------------------------
        for i in range(N + 1):  # N is last index in this loop
            x_arr[i, :] = es_solver.get(i, 'x')
            if i < N:
                u_arr[i, :] = es_solver.get(i, 'u')
                z_arr[i, :] = es_solver.get(i, 'z')
                sl_arr[i, :] = es_solver.get(i, 'sl')
                su_arr[i, :] = es_solver.get(i, 'su')
        x_arr[N, :] = es_solver.get(N, 'x')
        x_arr[:, 0] -= x_arr[0, 0]  # mitigate floating s variable

        tre_arr[:, ] = es_constraints.kamm_c((x_arr[:-1, ]).T, u_arr[:, ].T, kappa_disc[:-1].T).T
        p_arr[:, ] = es_constraints.p_drive(x_arr[:-1, ].T, u_arr[:, ].T).T

        f_late_arr[:, ] = es_constraints.f_late(x_arr[:, ].T, kappa_disc.T).T
        f_long_arr[:, ] = es_constraints.f_long(u_arr[:, ].T).T

        temp_cool_12_arr[:, ] = es_constraints.temp_cool_12(x_arr[:, ].T).T
        temp_cool_13_arr[:, ] = es_constraints.temp_cool_13(x_arr[:, ].T).T

        bat_loss[:, ] = es_constraints.bat_total_loss(x_arr[:-1, ].T, u_arr[:, ].T).T
        v_dc[:, ], Ri[:, ] = es_constraints.v_dc(x_arr[:, ].T).T, es_constraints.Ri(x_arr[:, ].T).T
        machine_loss[:, ] = es_constraints.machine_loss(x_arr[:-1, ].T, u_arr[:, ].T).T
        machine_loss_copper[:, ] = es_constraints.machine_loss_copper(x_arr[:-1, ].T, u_arr[:, ].T).T
        machine_loss_statoriron[:, ] = es_constraints.machine_loss_statoriron(x_arr[:-1, ].T, u_arr[:, ].T).T
        machine_loss_rotor[:, ] = es_constraints.machine_loss_rotor(x_arr[:-1, ].T, u_arr[:, ].T).T
        inverter_loss[:, ] = es_constraints.inverter_loss(x_arr[:-1, ].T, u_arr[:, ].T).T
        inverter_loss_switching[:, ] = es_constraints.inverter_loss_switching(x_arr[:-1, ].T, u_arr[:, ].T).T
        inverter_loss_conducting[:, ] = es_constraints.inverter_loss_conducting(x_arr[:-1, ].T, u_arr[:, ].T).T

        print('Number of machines assumed for results plot: 2')
        p_total_arr[:, ] = p_arr + 2 * inverter_loss + 2 * machine_loss + bat_loss

        self.x, self.u, self.z, self.sl, self.su = x_arr, u_arr, z_arr, sl_arr, su_arr
        self.tre, self.pwr, self.pwr_total, self.f_lat, self.f_lon, self.t_cool_12, self.t_cool_13 = \
            tre_arr, p_arr, p_total_arr, f_late_arr, f_long_arr, temp_cool_12_arr, temp_cool_13_arr

        self.bat_loss = bat_loss
        self.v_dc, self.Ri = v_dc, Ri
        self.machine_loss, self.machine_loss_copper, self.machine_loss_statoriron, self.machine_loss_rotor = \
            machine_loss, machine_loss_copper, machine_loss_statoriron, machine_loss_rotor
        self.inverter_loss, self.inverter_loss_switching, self.inverter_loss_conducting = \
            inverter_loss, inverter_loss_switching, inverter_loss_conducting

    def plot_results(self,
                     es_model: types.SimpleNamespace,
                     s_glo_offset: float = 0.0,
                     b_save_tikz: bool = False):
        """Plots extracted results and saves them as separate .tex-files (optional).

        :param es_model: optimization model (CasADi language)
        :param s_glo_offset: converts the local solver s-coordinate into the global s-coordinate of the race track
        including an offset from previous solver instances
        :param b_save_tikz: saves the plots as separate .tex-files

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            01.11.2020
        """

        # Pack battery losses -----------------------------------------------------------------------------------------
        es_fig, es_ax = plt.subplots(X_PLOTS_THERM, Y_PLOTS_THERM)
        es_ax[V_BAT_LOSS, H_BAT_LOSS].plot(self.x[:-1, 0] + s_glo_offset, self.bat_loss)  # battery loss
        es_ax[V_BAT_LOSS, H_BAT_LOSS].plot(self.x[:, 0] + s_glo_offset, self.x[:, 3] * 10)  # battery SOC
        es_ax[V_BAT_LOSS, H_BAT_LOSS].set_title('Battery losses')
        es_ax[V_BAT_LOSS, H_BAT_LOSS].set_ylabel('Power in kW')
        es_ax[V_BAT_LOSS, H_BAT_LOSS].legend(['total', 'SOC x 10'])

        # Pack battery temp --------------------------------------------------------------------------------------------
        es_ax[V_BAT_TEMP, H_BAT_TEMP].plot(self.x[:, 0] + s_glo_offset, self.x[:, 4])  # battery temp
        es_ax[V_BAT_TEMP, H_BAT_TEMP].plot(self.x[:, 0] + s_glo_offset, self.x[:, 8])  # battery cooling liquid temp
        es_ax[V_BAT_TEMP, H_BAT_TEMP].plot([self.x[0, 0] + s_glo_offset, self.x[-1, 0]], [es_model.temp_batt_max,
                                                                           es_model.temp_batt_max], 'r--')
        es_ax[V_BAT_TEMP, H_BAT_TEMP].set_title('Battery temperature')
        es_ax[V_BAT_TEMP, H_BAT_TEMP].set_ylabel('Temp. in °C')
        es_ax[V_BAT_TEMP, H_BAT_TEMP].legend(['bat', 'col'])

        # Pack MI temp -------------------------------------------------------------------------------------------------
        es_ax[V_INM_TEMP, H_INM_TEMP].plot(self.x[:, 0] + s_glo_offset, self.x[:, 5])  # machine temp
        es_ax[V_INM_TEMP, H_INM_TEMP].plot(self.x[:, 0] + s_glo_offset, self.x[:, 6])  # inverter temp
        es_ax[V_INM_TEMP, H_INM_TEMP].plot(self.x[:, 0] + s_glo_offset, self.x[:, 7])  # mach.-inv. cool. liqu. temp
        es_ax[V_INM_TEMP, H_INM_TEMP].plot([self.x[0, 0] + s_glo_offset, self.x[-1, 0]],
                                           [es_model.temp_mach_max, es_model.temp_mach_max], 'r--')
        es_ax[V_INM_TEMP, H_INM_TEMP].plot([self.x[0, 0] + s_glo_offset, self.x[-1, 0] + s_glo_offset],
                                           [es_model.temp_inv_max, es_model.temp_inv_max], 'r--')
        es_ax[V_INM_TEMP, H_INM_TEMP].set_title('MachInf temperature')
        es_ax[V_INM_TEMP, H_INM_TEMP].set_ylabel('Temp. in °C')
        es_ax[V_INM_TEMP, H_INM_TEMP].legend(['mac', 'inv', 'col'])

        # Pack battery temrinal ----------------------------------------------------------------------------------------
        es_ax[V_VDC, H_VDC].plot(self.x[:, 0] + s_glo_offset, self.v_dc)  # terminal voltage
        es_ax[V_VDC, H_VDC].plot(self.x[:, 0] + s_glo_offset, self.Ri * 1000)  # battery resistance

        es_ax[V_VDC, H_VDC].set_title('Terminal voltage')
        es_ax[V_VDC, H_VDC].set_ylabel('U in V, R in Ohm')
        es_ax[V_VDC, H_VDC].set_xlabel('Distance in m')
        es_ax[V_VDC, H_VDC].legend(['V_DC', 'R_i x 1000'])

        # Pack inverter losses -----------------------------------------------------------------------------------------
        es_ax[V_INV_LOSS, H_INV_LOSS].plot(self.x[:-1, 0] + s_glo_offset, self.inverter_loss)  # single inverter loss
        es_ax[V_INV_LOSS, H_INV_LOSS].plot(self.x[:-1, 0] + s_glo_offset, self.inverter_loss_switching)  # switching
        es_ax[V_INV_LOSS, H_INV_LOSS].plot(self.x[:-1, 0] + s_glo_offset, self.inverter_loss_conducting)  # conducting
        es_ax[V_INV_LOSS, H_INV_LOSS].set_title('Single inverter losses')
        es_ax[V_INV_LOSS, H_INV_LOSS].legend(['total', 'swi', 'con'])

        # Pack machine losses ------------------------------------------------------------------------------------------
        es_ax[V_MACH_LOSS, H_MACH_LOSS].plot(self.x[:-1, 0] + s_glo_offset, self.machine_loss)  # single machine loss
        es_ax[V_MACH_LOSS, H_MACH_LOSS].plot(self.x[:-1, 0] + s_glo_offset, self.machine_loss_copper)  # copper
        es_ax[V_MACH_LOSS, H_MACH_LOSS].plot(self.x[:-1, 0] + s_glo_offset,
                                             self.machine_loss_statoriron + self.machine_loss_rotor)  # stator & rotor
        es_ax[V_MACH_LOSS, H_MACH_LOSS].set_title('Single machine losses')
        es_ax[V_MACH_LOSS, H_MACH_LOSS].set_xlabel('Distance in m')
        es_ax[V_MACH_LOSS, H_MACH_LOSS].set_ylabel('Power in kW')
        es_ax[V_MACH_LOSS, H_MACH_LOSS].legend(['total', 'cop', 'sir'])

        if b_save_tikz:
            tikzplotlib.save('therm.tex')
            plt.show()

        # Pack driving dynamics ----------------------------------------------------------------------------------------
        es_fig_dyn, es_ax_dyn = plt.subplots(X_PLOTS_DYN, Y_PLOTS_DYN)
        es_ax_dyn[V_VEL].plot(self.x[:, 0] + s_glo_offset, self.x[:, 1])  # v
        np.savetxt('v_mrtp.csv', self.x[:, 1], '%.32f')
        es_ax_dyn[V_VEL].step(self.x[:-1, 0] + s_glo_offset, self.pwr)  # P
        es_ax_dyn[V_VEL].step(self.x[:-1, 0] + s_glo_offset, self.pwr_total)  # P including all powertrain losses
        es_ax_dyn[V_VEL].plot([self.x[0, 0] + s_glo_offset, self.x[-1, 0] + s_glo_offset],
                              [es_model.p_drive_max, es_model.p_drive_max],
                              'r--')
        es_ax_dyn[V_VEL].set_title('Velocity, power')
        es_ax_dyn[V_VEL].set_ylabel('mps, kW')
        es_ax_dyn[V_VEL].legend(['v', 'P_drive', 'P_total'])

        es_ax_dyn[V_FOR].step(self.x[:-1, 0] + s_glo_offset, self.u[:, 0])  # F drive
        es_ax_dyn[V_FOR].step(self.x[:-1, 0] + s_glo_offset, self.u[:, 1])  # F brake
        es_ax_dyn[V_FOR].step(self.x[:-1, 0] + s_glo_offset, (self.u[:, 0] * self.u[:, 1]) * 1000)  # F complementary
        es_ax_dyn[V_FOR].set_ylim([es_model.f_brake_min * 1.05, es_model.f_drive_max * 1.05])
        es_ax_dyn[V_FOR].plot([self.x[0, 0] + s_glo_offset, self.x[-1, 0] + s_glo_offset],
                              [es_model.f_drive_max, es_model.f_drive_max], 'r--')
        es_ax_dyn[V_FOR].plot([self.x[0, 0] + s_glo_offset, self.x[-1, 0] + s_glo_offset],
                              [es_model.f_brake_min, es_model.f_brake_min], 'r--')
        es_ax_dyn[V_FOR].set_title('Force')
        es_ax_dyn[V_FOR].set_ylabel('kN')
        es_ax_dyn[V_FOR].legend(['F_d', 'F_b', 'F_comp x 1000'])

        es_ax_dyn[V_TRE].step(self.x[:-1, 0] + s_glo_offset, self.tre[:, 0])  # Combined accel.
        # Slacks on upper bound of combined accel. constraints
        es_ax_dyn[V_TRE].step(self.x[:-1, 0] + s_glo_offset, self.su * 1e6)
        es_ax_dyn[V_TRE].plot([self.x[0, 0] + s_glo_offset, self.x[-1, 0]], [es_model.kamm_c_min, es_model.kamm_c_min], 'r--')
        es_ax_dyn[V_TRE].plot([self.x[0, 0] + s_glo_offset, self.x[-1, 0]], [es_model.kamm_c_max, es_model.kamm_c_max], 'r--')
        es_ax_dyn[V_FOR].set_xlabel('Distance in m')
        es_ax_dyn[V_TRE].set_title('Combined accel.')
        es_ax_dyn[V_TRE].legend(['tre', 'slc x 1e6'])

        if b_save_tikz:
            tikzplotlib.save('drive.tex')
            plt.show()

        plt.show()

    def show_statistics(self,
                        solver: AcadosOcpSolver):
        """Shows internal solver statistics, e.g., convergence rates and residuals.

        :param solver: acados solver object

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>
            Maximilian Bayerlein

        :Created on:
            01.11.2020
        """

        statistics = solver.get_stats('statistics')

        fig, axs = plt.subplots(3, 4)
        axs[0, 0].semilogy(statistics[0, :], statistics[1, :])
        axs[0, 0].set_title('Stationary residual')

        axs[0, 1].semilogy(statistics[0, :], statistics[2, :])
        axs[0, 1].set_title('Equational residual')

        axs[0, 2].semilogy(statistics[0, :], statistics[3, :])
        axs[0, 2].set_title('Inequational residual')

        axs[0, 3].semilogy(statistics[0, :], statistics[4, :])
        axs[0, 3].set_title('Complementary residual')

        axs[1, 0].plot(statistics[0, :], statistics[1, :])

        axs[1, 1].plot(statistics[0, :], statistics[2, :])

        axs[1, 2].plot(statistics[0, :], statistics[3, :])

        axs[1, 3].plot(statistics[0, :], statistics[4, :])

        axs[2, 1].plot(statistics[0, :], statistics[5, :])
        axs[2, 1].set_title('QP Status')

        axs[2, 2].plot(statistics[0, :], statistics[6, :])
        axs[2, 2].set_title('QP Iterations')

        plt.show()
