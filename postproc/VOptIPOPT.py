import casadi as cs
import os
import sys
import numpy as np
import tikzplotlib
from matplotlib import pyplot as plt

# own modules
mod_global_trajectory_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(mod_global_trajectory_path)

from src.def_es_model import es_model
from src.def_es_solver import es_solver
from src.get_track import get_track


class VOptIPOPT:

    def __init__(self,
                 laps: int,
                 TRACK: dict,
                 x0: np.ndarray,
                 b_dd_ref: bool):
        """Class to parse an ACADOS omptimization problem, prepared to be solved by HPIPM, for the IPOPT solver.
        Additionally, the parsed problem is solved by IPOPT.

        :param laps: number of race laps
        :param TRACK: track dictionary containing number of discretization points and length (see below for an example)
        :param x0: initial conditions
        :param b_dd_ref: switch to create driving dynamics reference speed profile or to solve energy strategy
        optimization problem

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            10.06.2020
        """

        self.sol_init(laps=laps,
                      TRACK=TRACK,
                      x0=x0,
                      b_dd_ref=b_dd_ref)

    def sol_init(self,
                 laps: int,
                 TRACK: dict,
                 x0: np.ndarray,
                 b_dd_ref: bool):
        """Solve an MRTP using ACADOS and subsequently run this script to get a direct solver comparison of
        IPOPT and HPIPM.
        Provide the information about the comparison data files below!

        :param laps: number of race laps
        :param TRACK: track dictionary containing number of discretization points and length (see below for an example)
        :param x0: initial conditions
        :param b_dd_ref: switch to create driving dynamics reference speed profile or to solve energy strategy
        optimization problem

        :Authors:
            Thomas Herrmann <thomas.herrmann@tum.de>

        :Created on:
            10.06.2020
        """
        f_v_ref = '../v_ref.csv'
        f_v_mrtp_acados = 'example/v_mrtp.csv'

        # --- Import SINGLE race lap with variable mesh
        [self.s_ref, self.kappa_disc, self.s_track] = get_track(laps=1,
                                                                TRACK=TRACK,
                                                                b_plot=False)

        # tile delta_s
        self.s_ref = np.tile(self.s_ref, laps)

        k_tail = np.tile(self.kappa_disc[1:], (laps - 1, 1))
        self.kappa_disc = np.concatenate((self.kappa_disc, k_tail))

        # calculate global s coordinate starting from 0
        s_global = np.insert(np.cumsum(self.s_ref), 0, 0)

        # Get number of discretization points
        N = len(self.s_ref)

        # load driving dynamics reference speed profile
        v_ref = np.loadtxt(f_v_ref, dtype=np.float64)

        tmp_v_ref = v_ref
        for i in range(0, laps - 1):
            v_ref = np.concatenate((v_ref, tmp_v_ref[1:]))

        # Import defined CasADi-model and constraints to solve using IPOPT
        model, constraint = es_model(b_dd_ref=b_dd_ref,
                                     b_battery_simple=False,
                                     b_inverter_simple=False,
                                     b_machine_simple=False)

        # Import defined constraints in CasADi modelling language
        acados_solver = es_solver(model=model,
                                  constraint=constraint,
                                  time_steps=self.s_ref,
                                  hessian_method="EXACT",
                                  b_dd_ref=b_dd_ref,
                                  x0=x0)

        # NLP for IPOPT
        x = model.x
        u = model.u
        # z = model.z
        p = model.p
        x0 = x0

        xdot = model.f_expl_expr_ipopt[:-1]
        # zdot = model.f_expl_expr_ipopt[-1]

        # initialize x-vector with +-infinity
        lbx_ac = np.ones((model.x.size()[0], )) * (- np.inf)
        ubx_ac = np.ones((model.x.size()[0], )) * np.inf
        # Get indices of state constraints
        idxbx = acados_solver.acados_ocp.constraints.idxbx
        # Fill state boundary vectors
        lbx_ac[idxbx] = acados_solver.acados_ocp.constraints.lbx
        ubx_ac[idxbx] = acados_solver.acados_ocp.constraints.ubx

        # initialize u-vector with +-infinity
        lbu_ac = np.ones((model.u.size()[0],)) * (- np.inf)
        ubu_ac = np.ones((model.u.size()[0],)) * np.inf
        # Get indices of input constraints
        idxbu = acados_solver.acados_ocp.constraints.idxbu
        # Fill input boundary vectors
        lbu_ac[idxbu] = acados_solver.acados_ocp.constraints.lbu
        ubu_ac[idxbu] = acados_solver.acados_ocp.constraints.ubu

        g_ac = constraint.expr
        lbg_ac = acados_solver.acados_ocp.constraints.lh
        ubg_ac = acados_solver.acados_ocp.constraints.uh

        # Sum over time, TODO: use algebraic variable here
        y_res = x - acados_solver.acados_ocp.cost.yref[:x.size()[0]]
        u_res = u - acados_solver.acados_ocp.cost.yref[x.size()[0]:x.size()[0] + u.size()[0]]
        J_sym = cs.sum1(1e4 * (1 / x[1]) ** 2
                        + cs.mtimes(
            cs.mtimes(y_res.T, acados_solver.acados_ocp.cost.W[:x.size()[0], :x.size()[0]]), y_res)
                        + cs.mtimes(
            cs.mtimes(u_res.T, acados_solver.acados_ocp.cost.W[
                          x.size()[0]:x.size()[0] + u.size()[0], x.size()[0]:x.size()[0] + u.size()[0]]), u_res)
                        )

        ################################################################################################################
        # ERK4 integrator
        ################################################################################################################

        # Fixed step Runge-Kutta 4 integrator
        M = acados_solver.acados_ocp.solver_options.sim_method_num_steps  # RK4 steps per interval
        fInt = cs.Function('fInt', [x, u], [xdot, J_sym])
        X0 = cs.MX.sym('X0', model.x.size()[0])
        U = cs.MX.sym('U', model.u.size()[0])
        ds = cs.MX.sym('ds', 1)
        X = X0
        Q = 0
        for j in range(M):  # TODO: implement IRK4 here
            k1, k1_q = fInt(X, U)
            k2, k2_q = fInt(X + ds / M / 2 * k1, U)
            k3, k3_q = fInt(X + ds / M / 2 * k2, U)
            k4, k4_q = fInt(X + ds / M * k3, U)
            X = X + ds / M / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            Q = Q + ds / M / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)
        F = cs.Function('F', [X0, U, ds], [X, Q], ['x0', 'p', 'ds'], ['xf', 'qf'])

        # Empty NLP
        w = []
        w0 = []
        lbw = []
        ubw = []
        J = 0
        g = []
        lbg = []
        ubg = []
        par_vec = []

        # "Lift" initial conditions
        Xk = cs.MX.sym('X0', model.x.size()[0])
        w += [Xk]
        lbw += list(lbx_ac)
        ubw += list(ubx_ac)
        w0 += list(x0)

        # Initial state constraint
        g += [Xk]
        lbg += list(x0)
        ubg += list(x0)

        # Convert ACADOS nonlinear expressions to CasADi nonlinear expressions
        g_ac2cas = {}
        for ig in range(0, g_ac.shape[0]):
            g_ac2cas["{0}".format(ig)] = cs.Function('g_' + str(ig), [x, u, p], [g_ac[ig]])

        # Formulate the multiple-shooting NLP
        for k in range(self.kappa_disc.size - 1):
            # New NLP variable for the control
            Uk = cs.MX.sym('U_' + str(k), model.u.size()[0])
            w += [Uk]
            lbw += list(lbu_ac)
            ubw += list(ubu_ac)
            w0 += list(acados_solver.get(k, 'u'))

            # Integrate till the end of the variable sized interval
            Fk = F(x0=Xk, p=Uk, ds=self.s_ref[k])
            Xk_end = Fk['xf']
            J = J + Fk['qf']

            # New NLP variable for state at end of interval
            Xk = cs.MX.sym('X_' + str(k + 1), model.x.size()[0])
            w += [Xk]
            lbw += list(lbx_ac)
            ubw += list(ubx_ac)
            w0 += list((x0[0], v_ref[k], x0[2], x0[3], x0[4], x0[5], x0[6], x0[7], x0[8]))

            # Add equality constraint
            g += [Xk_end - Xk]
            lbg += [0] * model.x.size()[0]
            ubg += [0] * model.x.size()[0]

            pk = cs.MX.sym('p_' + str(k), model.p.size()[0])
            par_vec += [pk]
            # Append nonlinear constraints to every shooting node
            for ig in range(0, g_ac.shape[0]):
                g += [g_ac2cas[str(ig)](Xk, Uk, pk)]
                lbg += [lbg_ac[ig]]
                ubg += [ubg_ac[ig]]

        # Create an NLP solver
        prob = {'f': J, 'x': cs.vertcat(*w), 'g': cs.vertcat(*g), 'p': cs.vertcat(*par_vec)}

        opts_IPOPT = {"expand": False,
                      "ipopt.print_level": 5,
                      "ipopt.max_iter": 1000,
                      "ipopt.tol": 1e-3,
                      "ipopt.nlp_scaling_method": 'none',
                      "verbose": False}

        solver = cs.nlpsol('solver', 'ipopt', prob, opts_IPOPT)

        # Solve the NLP
        sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg, p=self.kappa_disc[:-1].T)

        w_opt = sol['x'].full().flatten()

        # Extract solution
        f_drive_o = w_opt[model.x.shape[0]::model.x.shape[0]+model.u.shape[0]]
        f_brake_o = w_opt[model.x.shape[0]+1::model.x.shape[0]+model.u.shape[0]]
        x_opt = [list(x0.reshape((x0.shape[0], 1)))]
        for k in range(self.kappa_disc.size - 1):
            Fk = F(x0=x_opt[-1], p=[f_drive_o[k], f_brake_o[k]], ds=self.s_ref[k])
            x_opt += [Fk['xf'].full()]

        # s_pm, v, t_pm, soc_batt, temp_batt, temp_mach, temp_inv, temp_cool_mi, temp_cool_b
        s_pm_o = np.concatenate([r[0] for r in x_opt])
        v_o = np.concatenate([r[1] for r in x_opt])
        t_pm_o = np.concatenate([r[2] for r in x_opt])
        soc_batt_o = np.concatenate([r[3] for r in x_opt])
        temp_batt_o = np.concatenate([r[4] for r in x_opt])
        temp_mach_o = np.concatenate([r[5] for r in x_opt])
        temp_inv_o = np.concatenate([r[6] for r in x_opt])
        temp_cool_mi_o = np.concatenate([r[7] for r in x_opt])
        temp_cool_b_O = np.concatenate([r[8] for r in x_opt])

        # --- Load ACADOS + HPIPM solution
        v_ac = np.loadtxt(f_v_mrtp_acados)

        plt.figure()
        plt.subplot(4, 1, 1)
        plt.plot(s_global, v_o)
        plt.plot(s_global, v_ac)
        plt.legend(['IPOPT', 'HPIPM'])
        plt.xlabel('s in m')
        plt.xlabel('v in m/s')
        plt.subplot(4, 1, 2)
        plt.plot(soc_batt_o)
        plt.ylabel('SOC')
        plt.subplot(4, 1, 3)
        plt.plot(temp_batt_o)
        plt.ylabel('T_Batt in °C')
        plt.subplot(4, 1, 4)
        plt.plot(temp_mach_o)
        plt.ylabel('T_Machine in °C')
        tikzplotlib.save('mrtp_IPOPT.tex')
        plt.figure()
        plt.plot(f_drive_o)
        plt.plot(f_brake_o)
        print('NRMSE suboptimality for velocity in MRTP: IPOPT vs HPIPM [%]',
              np.sqrt(np.sum((v_ac - v_o) ** 2) / v_ac.size) / (np.max(v_o) - np.min(v_o)) * 100)
        plt.show()
        '''
        self.trj = cs.Function('trj',
                               [x, param_vec],
                               [F_p, tire_cst1, acc, pwr_cst],
                               ['x', 'kappa_param'],
                               ['F_p', 'tire_cst', 'acc', 'pwr_cst'])
        '''

        return solver.stats()['t_proc_total']


if __name__ == "__main__":
    """Run IPOPT solver comparison."""

    # number of race laps
    laps_ = 12

    # track information
    TRACK_ = {"Name": 'Monteblanco', "discr": 298, "v0": 45, "Length": 2382}

    # initial conditions
    # s_pm, v, t_pm, soc_batt, temp_batt, temp_mach, temp_inv, temp_cool_mi, temp_cool_b
    x0_ = np.array([1, TRACK_["v0"], 0, 0.5, 35, 35, 35, 35, 35])

    VOptIPOPT(laps=laps_,
              TRACK=TRACK_,
              x0=x0_,
              b_dd_ref=False)
