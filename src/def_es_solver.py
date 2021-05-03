import types
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import scipy.linalg
import numpy as np


def es_solver(model: types.SimpleNamespace,
              constraint: types.SimpleNamespace,
              time_steps: np.ndarray,
              hessian_method: str,
              x0: np.array,
              nlp_steplen: float = 1.0,
              b_dd_ref: bool = False) -> AcadosOcpSolver:
    """Creates the optimization solver using acados modeling language.

    :param model: optimization model defined using CasADi modeling language
    :param constraint: optimization constraints defined using CasADi modeling language
    :param time_steps: discrete time_steps to approximate the DAE system
    :param hessian_method: can be 'GAUSS_NEWTON' (less accurate) or 'HESSIAN' (higher computation time)
    :param x0: initial conditions of ODEs
    :param nlp_steplen: initial NLP solver steplength (default: 100 %)
    :param b_dd_ref: switch to create reference velocity profile

    :return: acados_solver: acados OCP solver object

    :Authors:
        Thomas Herrmann <thomas.herrmann@tum.de>
        Maximilian Bayerlein

    :Created on:
        01.06.2020
    """

    # create render arguments
    ocp = AcadosOcp()

    # ------------------------------------------------------------------------------------------------------------------
    # Define ACADOS OCP model ------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = []
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.z = model.z
    model_ac.p = model.p
    model_ac.name = 'es_model'
    ocp.model = model_ac

    # Set dimensions ---------------------------------------------------------------------------------------------------
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    nz = model.z.size()[0]
    ny = nx + nu + nz
    ny_e = nx
    ocp.dims.N = len(time_steps)
    tf = np.sum(time_steps)

    # Set cost for states ----------------------------------------------------------------------------------------------
    # s, v, t, soc_batt, temp_batt, temp_mach, temp_inv, temp_cool_mi, temp_cool_b
    Q = np.diag([0, 0, 0, 1e-2, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5])
    # set cost for terminal states -------------------------------------------------------------------------------------
    Qe = np.diag([0, 0, 0, 1e-2, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5])

    # Set cost for controls --------------------------------------------------------------------------------------------
    R = np.eye(nu)
    R[0, 0] = 1e-2
    R[1, 1] = 1e-3

    # Set cost for algebraic variable ----------------------------------------------------------------------------------
    P = np.eye(nz)
    P[0, 0] = 1

    # Objective function cost type -------------------------------------------------------------------------------------
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    # Construct weight matrix ------------------------------------------------------------------------------------------
    ocp.cost.W = scipy.linalg.block_diag(Q, R, P)
    ocp.cost.W_e = Qe

    # Dims matrices ----------------------------------------------------------------------------------------------------
    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    Vx_e = np.zeros((ny_e, nx))
    Vx_e[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = Vx_e

    Vu = np.zeros((ny, nu))
    Vu[-3, 0] = 1
    Vu[-2, 1] = 1
    ocp.cost.Vu = Vu

    Vz = np.zeros((ny, nz))
    Vz[-1, 0] = 1
    ocp.cost.Vz = Vz

    # Slack variable weights -------------------------------------------------------------------------------------------
    ocp.cost.zl = 0 * np.ones((4, ))  # cost on linear slack variable (lower)
    ocp.cost.Zl = 0 * np.ones((4, ))  # cost on quadratic slack variable (lower)
    ocp.cost.zu = 1000 * np.ones((4, ))   # cost on linear slack variable (upper)
    ocp.cost.Zu = 10 * np.ones((4, ))  # cost on quadratic slack variable (upper)

    # References in cost term residual terms ---------------------------------------------------------------------------
    if b_dd_ref:
        ocp.cost.yref = np.array([0, 0, 0, 0.01, model.temp_batt_min, model.temp_mach_min, model.temp_inv_min,
                                  model.temp_cool_mach_inv_min, model.temp_cool_batt_min,
                                  5, -10, 0])
        ocp.cost.yref_e = np.array([0, 0, 0, 0.01, model.temp_batt_min, model.temp_mach_min, model.temp_inv_min,
                                  model.temp_cool_mach_inv_min, model.temp_cool_batt_min])
    else:
        ocp.cost.yref = np.array([0, 0, 0, 1, model.temp_batt_min, model.temp_mach_min, model.temp_inv_min,
                                  model.temp_cool_mach_inv_min, model.temp_cool_batt_min,
                                  5, -10, 0])
        ocp.cost.yref_e = np.array([0, 0, 0, 1, model.temp_batt_min, model.temp_mach_min, model.temp_inv_min,
                                   model.temp_cool_mach_inv_min, model.temp_cool_batt_min])

    # Set state constraints --------------------------------------------------------------------------------------------
    ocp.constraints.lbx = np.array([model.v_min, model.soc_min,
                                    model.temp_batt_min, model.temp_mach_min, model.temp_inv_min,
                                    model.temp_cool_mach_inv_min, model.temp_cool_batt_min])
    ocp.constraints.ubx = np.array([model.v_max, model.soc_max,
                                    model.temp_batt_max, model.temp_mach_max, model.temp_inv_max,
                                    model.temp_cool_mach_inv_max, model.temp_cool_batt_max])
    ocp.constraints.idxbx = np.array([1, 3, 4, 5, 6, 7, 8])
    ocp.constraints.lbx_e = np.array([model.v_min, model.soc_min,
                                      model.temp_batt_min, model.temp_mach_min, model.temp_inv_min,
                                      model.temp_cool_mach_inv_min, model.temp_cool_batt_min])
    ocp.constraints.ubx_e = np.array([model.v_max, model.soc_max,
                                      model.temp_batt_max, model.temp_mach_max, model.temp_inv_max,
                                      model.temp_cool_mach_inv_max, model.temp_cool_batt_max])
    ocp.constraints.idxbx_e = np.array([1, 3, 4, 5, 6, 7, 8])

    # Set control constraints ------------------------------------------------------------------------------------------
    ocp.constraints.lbu = np.array([model.f_drive_min, model.f_brake_min])
    ocp.constraints.ubu = np.array([model.f_drive_max, model.f_brake_max])
    ocp.constraints.idxbu = np.array([0, 1])

    # Set nonlinear path constraints and boundaries --------------------------------------------------------------------
    model_ac.con_h_expr = constraint.expr
    ocp.constraints.lh = np.array(
        [
            model.kamm_c_min,
            model.kamm_c_min,
            model.kamm_c_min,
            model.kamm_c_min,
            model.p_drive_min
        ]
    )
    ocp.constraints.uh = np.array(
        [
            model.kamm_c_max,
            model.kamm_c_max,
            model.kamm_c_max,
            model.kamm_c_max,
            model.p_drive_max
        ]
    )
    # In case of driving dynamics reference, activate f_act-constraint
    if b_dd_ref:
        ocp.constraints.lh = np.append(ocp.constraints.lh, model.f_act_min)
        ocp.constraints.uh = np.append(ocp.constraints.uh, model.f_act_max)
    else:
        pass

    # Soften combined acceleration constraints
    ocp.constraints.idxsh = np.array([0, 1, 2, 3])

    # Set solver settings ----------------------------------------------------------------------------------------------
    # time steps
    ocp.solver_options.tf = tf
    # QP solver
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.regularize_method = 'CONVEXIFY'
    # QP solver type
    ocp.solver_options.nlp_solver_type = "SQP"
    # Hessian approximation method
    ocp.solver_options.hessian_approx = hessian_method

    ocp.solver_options.integrator_type = "IRK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3

    # ocp.solver_options.qp_solver_cond_N = ocp.dims.N

    # NLP step length, can be adapted by built-in function '.options_set'
    ocp.solver_options.nlp_solver_step_length = nlp_steplen

    # QP and NLP max. iter
    if b_dd_ref:
        ocp.solver_options.nlp_solver_max_iter = 500
    else:
        ocp.solver_options.nlp_solver_max_iter = 10
    ocp.solver_options.qp_solver_iter_max = 50

    # NLP tolerances
    ocp.solver_options.nlp_solver_tol_eq = 1e0
    ocp.solver_options.nlp_solver_tol_ineq = 1e0
    ocp.solver_options.nlp_solver_tol_stat = 1e0
    ocp.solver_options.nlp_solver_tol_comp = 1e0

    # set variable sized grid
    ocp.solver_options.time_steps = time_steps

    # set initial value
    ocp.constraints.x0 = x0

    # set dummy track kappa parameter
    ocp.parameter_values = np.array([0])

    # Create solver ----------------------------------------------------------------------------------------------------
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")

    return acados_solver
