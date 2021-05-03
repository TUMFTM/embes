import casadi as cs
import types
import os
import configparser
import json
import sys
import copy

module_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
parser = configparser.ConfigParser()

# use INTERNAL repository 'mod_global_trajectory'
if parser.read(os.path.join(module_path, "../mod_global_trajectory", "params", "db.ini")):
    print("\n[INFO] *** Using INTERNAL development config and model descriptions. ***\n")
    mod_global_trajectory_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) \
        + '/mod_global_trajectory'
    sys.path.append(mod_global_trajectory_path)
    import opt_mintime_traj as pwr_src

# use EXTERNAL repository 'global_racetrajectory_optimization'
elif parser.read(os.path.join(module_path, "external/global_racetrajectory_optimization",
                              "params", "racecar.ini")):
    print("\n[INFO] *** Using EXTERNAL config and model descriptions. ***\n")
    mod_global_trajectory_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) \
        + '/external/global_racetrajectory_optimization'
    sys.path.append(mod_global_trajectory_path)
    import external.global_racetrajectory_optimization.opt_mintime_traj.powertrain_src.src as pwr_src
    # adapt module path to make it treatable as if it was an internal import (see above)
    pwr_src = pwr_src.opt_mintime_traj
else:
    raise ValueError('Specified config file does not exist or is empty!')


def es_model(b_dd_ref: bool = False,
             b_battery_simple: bool = False,
             b_machine_simple: bool = False,
             b_inverter_simple: bool = False) -> tuple:
    """Creates the optimization model using CasADi modeling language and prepares it for the use in acados.

    :param b_dd_ref: switch to enable the calculation of the reference velocity profile
    :param b_battery_simple: switch to enable a simple loss model of the battery
    :param b_machine_simple: switch to enable a simple loss model of the machine
    :param b_inverter_simple: switch to enable a simple loss model of the inverter

    :return: model: optimization model\n
        constraint: optimization constraints

    :Authors:
        Thomas Herrmann <thomas.herrmann@tum.de>
        Maximilian Bayerlein

    :Created on:
        01.06.2020
    """

    # ------------------------------------------------------------------------------------------------------------------
    # Load vehicle parameter file --------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    pars = dict()

    # powertrain parameters
    pars["pwr"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'pwr_params_mintime'))
    # general parameters
    pars["gen"] = json.loads(parser.get('GENERAL_OPTIONS', 'veh_params'))
    # vehicle parameters
    pars["veh"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'vehicle_params_mintime'))
    # tire parameters
    pars["tre"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'tire_params_mintime'))
    # mintime parameters
    pars["opt"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'optim_opts_mintime'))

    # Parameter unit conversions
    pars["gen"]["mass"] *= 0.001  # tons
    pars["gen"]["dragcoeff"] *= 0.001

    # ------------------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    # OCP --------------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    # model
    model = types.SimpleNamespace()
    model.name = 'es_model'
    # constraints
    constraint = types.SimpleNamespace()

    # ------------------------------------------------------------------------------------------------------------------
    # State Variables --------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    # distance [m]
    s_var = cs.SX.sym('s_var')

    # velocity [m/s]
    v_var = cs.SX.sym('v_var')

    # time [s]
    t_var = cs.SX.sym('t_var')

    # --- Powertrain ---------------------------------------------------------------------------------------------------
    # --- Battery
    # Initialization with simple or detailed loss model
    pars_bat = copy.deepcopy(pars)
    pars_bat["pwr"]["simple_loss"] = b_battery_simple
    # SOC [-]
    battery = pwr_src.powertrain_src.src.Battery.BattModel(pwr_pars=pars_bat["pwr"])

    # Temperature [°C]
    battery.temp_batt = battery.temp_batt_n

    # --- Machine
    # Initialization
    pars_em = copy.deepcopy(pars)
    pars_em["pwr"]["simple_loss"] = b_machine_simple
    machine = pwr_src.powertrain_src.src.EMachine.EMachineModel(pwr_pars=pars_em["pwr"])
    # Electric machine temperature [°C]
    machine.temp_mot = machine.temp_mot_n

    # Inverter
    pars_in = copy.deepcopy(pars)
    pars_in["pwr"]["simple_loss"] = b_inverter_simple
    inverter = pwr_src.powertrain_src.src.Inverter.InverterModel(pwr_pars=pars_in["pwr"])
    # Temperature [°C]
    inverter.temp_inv = inverter.temp_inv_n

    # Temperature of the motor and inverter cooling circuit [°C]
    radiators = pwr_src.powertrain_src.src.Radiators.RadiatorModel(pwr_pars=pars["pwr"])
    radiators.temp_cool_mi = radiators.temp_cool_mi_n

    # Temperature of the battery cooling circuit [°C]
    radiators.temp_cool_b = radiators.temp_cool_b_n

    # --- State vector -------------------------------------------------------------------------------------------------
    x = cs.vertcat(s_var, v_var, t_var, battery.soc_batt,
                   battery.temp_batt, machine.temp_mot, inverter.temp_inv,
                   radiators.temp_cool_mi, radiators.temp_cool_b)

    # STATE BOUNDARIES -------------------------------------------------------------------------------------------------

    model.v_min = 1.0  # min. velocity [m/s]
    model.v_max = 100.0  # max. velocity [m/s]
    model.soc_min = 0.0  # min. SOC [-]
    model.soc_max = 1.0  # max. SOC [-]
    model.temp_batt_min = pars["pwr"]["T_env"]  # min. battery temperature [°C]
    model.temp_batt_max = pars["pwr"]["temp_batt_max"]  # max. battery temperature [°C]
    model.temp_mach_min = pars["pwr"]["T_env"]  # min. engine temperature [°C]
    model.temp_mach_max = pars["pwr"]["temp_mot_max"]  # max. engine temperature [°C]
    model.temp_inv_min = pars["pwr"]["T_env"]  # min. inverter temperature [°C]
    model.temp_inv_max = pars["pwr"]["temp_inv_max"]  # max. inverter temperature [°C]
    model.temp_cool_mach_inv_min = pars["pwr"]["T_env"]  # min. machine and inverter cooling circuit temperature [°C]
    model.temp_cool_mach_inv_max = pars["pwr"]["temp_inv_max"]  # max. mach.-inv. cooling circuit temperature [°C]
    model.temp_cool_batt_min = pars["pwr"]["T_env"]  # min. battery cooling circuit temperature [°C]
    model.temp_cool_batt_max = pars["pwr"]["temp_batt_max"]  # max. battery cooling circuit temperature [°C]

    # ------------------------------------------------------------------------------------------------------------------
    # Derivatives of State Variables -----------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    sdot = cs.SX.sym("sdot")
    vdot = cs.SX.sym("vdot")
    tdot = cs.SX.sym("tdot")
    soc_batt_dot = cs.SX.sym("soc_batt_dot")
    temp_batt_dot = cs.SX.sym("temp_batt_dot")
    temp_mach_dot = cs.SX.sym("temp_mach_dot")
    temp_inv_dot = cs.SX.sym("temp_inv_dot")
    temp_cool_mi_dot = cs.SX.sym("temp_cool_mi_dot")
    temp_cool_b_dot = cs.SX.sym("temp_cool_b_dot")
    xdot = cs.vertcat(sdot, vdot, tdot, soc_batt_dot,
                      temp_batt_dot, temp_mach_dot, temp_inv_dot,
                      temp_cool_mi_dot, temp_cool_b_dot)

    # ------------------------------------------------------------------------------------------------------------------
    # Control Variables ------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    # positive longitudinal drive force [kN]
    f_drive_var = cs.SX.sym('f_drive_var')

    # negative longitudinal force (brake) [kN]
    f_brake_var = cs.SX.sym('f_brake_var')

    # --- Control vector -----------------------------------------------------------------------------------------------
    u = cs.vertcat(f_drive_var, f_brake_var)

    # CONTROL BOUNDARIES -----------------------------------------------------------------------------------------------

    model.f_drive_min = 0.0  # min. longitudinal drive force [kN]
    model.f_drive_max = pars["veh"]["f_drive_max"] / 1000  # max. longitudinal drive force [kN]
    model.f_brake_min = - pars["veh"]["f_brake_max"] / 1000  # min. longitudinal brake force [kN]
    model.f_brake_max = 0.0  # max. longitudinal brake force [kN]

    # ------------------------------------------------------------------------------------------------------------------
    # Algebraic Variable -----------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    delta_t_alg = cs.SX.sym('delta_t_alg')
    delta_t_alg_s = 100  # algebraic scalar multiplier

    # --- Algebraic variables vector
    z = cs.vertcat(delta_t_alg)

    # ------------------------------------------------------------------------------------------------------------------
    # Parameters -------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    # discrete kappa values [rad/m]
    kappa_disc_par = cs.SX.sym('kappa_disc_par')

    # ------------------------------------------------------------------------------------------------------------------
    # Vehicle Dynamics Equations ---------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    # Drag force [kN]
    f_drag = pars["gen"]["dragcoeff"] * v_var ** 2

    # Rolling resistance force [kN]
    f_roll = pars["gen"]["mass"] * pars["gen"]["g"] * pars["tre"]["c_roll"]

    # Static normal tire force [kN]
    f_down_stat = pars["gen"]["mass"] * pars["gen"]["g"]

    # Dynamic normal tire force (aerodynamic downforces) [kN]
    f_down_dyn = 0.25 * (pars["veh"]["liftcoeff_front"] + pars["veh"]["liftcoeff_rear"]) * (v_var ** 2) * 0.001

    # Sum of normal tire forces [kN]
    f_down = f_down_stat + f_down_dyn

    # Sum of lateral tire forces [kN]
    f_lat = kappa_disc_par * pars["gen"]["mass"] * (v_var ** 2)

    # Sum of longitudinal tire forces [kN]
    f_long = f_drive_var + f_brake_var

    # Combined acceleration limit [-], similar to Kamm's circle but as a diamond
    KAMM_INFLATE = 100
    kamm_c_1 = (
                       (f_long / (pars["opt"]["mue"] * f_down))
                       + (f_lat / (pars["opt"]["mue"] * f_down))
               ) * KAMM_INFLATE
    kamm_c_2 = (
                       (f_long / (pars["opt"]["mue"] * f_down))
                       - (f_lat / (pars["opt"]["mue"] * f_down))
               ) * KAMM_INFLATE
    kamm_c_3 = (
                       - (f_long / (pars["opt"]["mue"] * f_down))
                       - (f_lat / (pars["opt"]["mue"] * f_down))
               ) * KAMM_INFLATE
    kamm_c_4 = (
                       - (f_long / (pars["opt"]["mue"] * f_down))
                       + (f_lat / (pars["opt"]["mue"] * f_down))
               ) * KAMM_INFLATE

    # Vehicle Power [kW]
    p_drive = f_drive_var * v_var

    # Actor restriction: no simultaneous braking and acceleration [kN^2]
    f_act = f_drive_var * f_brake_var

    # ------------------------------------------------------------------------------------------------------------------
    # Powertrain Dynamics Equations ------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # --- Machines
    # Initialize states of the electric machine symbolically
    machine.get_states(f_drive=f_drive_var * 1000,
                       v=v_var)

    # Machine loss (single) [kW]
    machine.get_loss(p_wheel=p_drive)

    # Calculate total power loss for all electric machines in vehicle [kW]
    machine.get_machines_cum_losses()

    # --- Inverters
    # Calculation of inverter internal power loss
    inverter.get_loss(i_eff=machine.i_eff,
                      v_dc=battery.v_dc,
                      p_out_inv=machine.p_input)

    # Cumulated losses of all inverters [kW]
    inverter.get_inverters_cum_losses()

    # --- Battery
    # Here, we distinguish between simple or detailed loss models as they need different inputs

    # get variable resistance with overwritten battery_temp-variable (temp_batt_n)
    battery.internal_resistance()

    # Calculation of battery internal power
    if pars_bat["pwr"]["simple_loss"] is False:
        battery.battery_loss(p_des=p_drive,
                             p_loss_mot=machine.p_loss_total_all_machines,
                             p_loss_inv=inverter.p_loss_total_all_inverters,
                             p_in_inv=inverter.p_loss_total_all_inverters + machine.p_loss_total_all_machines + p_drive)
    else:
        # Case that machine and inverter are also simple models
        try:
            battery.battery_loss(p_des=p_drive,
                                 p_loss_mot=machine.p_loss_total_all_machines,
                                 p_loss_inv=inverter.p_loss_total_all_inverters,
                                 p_in_inv=inverter.p_in_inv)

        # Case that machine and inverter are detailed models and the battery shall be a simple model
        except:
            battery.battery_loss(p_des=p_drive,
                                 p_loss_mot=machine.p_loss_total_all_machines,
                                 p_loss_inv=inverter.p_loss_total_all_inverters,
                                 p_in_inv=inverter.p_loss_total + machine.p_loss_total + p_drive * 0.5)

    # Intermediate temperatures motor and inverter circuit) [°C]
    radiators.get_intermediate_temps(temp_inv=inverter.temp_inv,
                                     r_inv=inverter.r_inv)

    # ------------------------------------------------------------------------------------------------------------------
    # DERIVATIVES ------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # derivative of position (ds/ds) [s]
    ds = 1

    # derivative of time (dt/ds) [s/m]
    dt = 1 / v_var

    # derivative of velocity (dv/ds) [1/s]
    dv = dt * (f_long - f_drag - f_roll) / pars["gen"]["mass"]

    # Temperature increment of electric machine [°C/m]
    machine.get_increment(sf=dt,
                          temp_cool_12=radiators.temp_cool_12,
                          temp_cool_13=radiators.temp_cool_13)

    # Temperature increment of inverter [°C/m]
    inverter.get_increment(sf=dt,
                           temp_cool_mi=radiators.temp_cool_mi,
                           temp_cool_12=radiators.temp_cool_12)

    # Battery temperature increment [°C/m]
    battery.get_increment(sf=dt,
                          temp_cool_b=radiators.temp_cool_b)

    # Temperature increment of machine and inverter fluid [°C/m]
    radiators.get_increment_mi(sf=dt,
                               temp_mot=machine.temp_mot,
                               temp_inv=inverter.temp_inv,
                               r_inv=inverter.r_inv,
                               r_machine=machine.r_machine)

    # Temperature increment of battery fluid [°C/m]
    radiators.get_increment_b(sf=dt,
                              temp_batt=battery.temp_batt,
                              temp_cool_b=radiators.temp_cool_b,
                              R_eq_B_inv=battery.r_batt_inverse)

    # State of charge increment [1/m]
    battery.get_soc(sf=dt)

    # --- Derivatives vector
    dx = cs.vertcat(ds, dv, dt, battery.dsoc,
                    battery.dtemp, machine.dtemp, inverter.dtemp,
                    radiators.dtemp_cool_mi, radiators.dtemp_cool_b)

    # ------------------------------------------------------------------------------------------------------------------
    # Nonlinear CONSTRAINTS --------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    # path constraint: Combined acceleration
    model.kamm_c_min = -1 * KAMM_INFLATE
    model.kamm_c_max = 1 * KAMM_INFLATE

    # path constraint: Limited engine power
    model.p_drive_min = 0
    model.p_drive_max = pars["veh"]["power_max"] / 1000  # max. provided engine power [kW]

    # complementary path constraint, only necessary in energy-UN-constrained case
    model.f_act_min = 0
    model.f_act_max = 0

    constraint.p_drive = cs.Function("p_drive", [x, u], [p_drive])
    constraint.kamm_c = cs.Function("kamm_c", [x, u, kappa_disc_par], [kamm_c_1])
    constraint.f_act = cs.Function("f_act", [x, u], [f_act])
    constraint.f_late = cs.Function("f_late", [x, kappa_disc_par], [f_lat])
    constraint.f_long = cs.Function("f_long", [u], [f_long])

    # calculate driving dynamics reference with f_act-constraint being active
    if b_dd_ref:
        constraint.expr = cs.vertcat(kamm_c_1, kamm_c_2, kamm_c_3, kamm_c_4, p_drive, f_act)
    # calc energy strategy without f_act-constraint as its formulation is non-convex and therefore reduces
    # computation time enormously. If the resulting Energy Strategy otimization problem reaches the SOC- or a
    # thermodynamic constraint, f_act is not necessary. F_drive and F_brake are then actuated in an optimal way to
    # not overstress the powertrain. Through this, f_act can be omitted as the optimal solution stays unique.
    else:
        constraint.expr = cs.vertcat(kamm_c_1, kamm_c_2, kamm_c_3, kamm_c_4, p_drive)

    # ------------------------------------------------------------------------------------------------------------------
    # Debug functions --------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    constraint.bat_total_loss = cs.Function("bat_total_loss", [x, u], [battery.p_loss_total])
    constraint.v_dc = cs.Function("v_dc", [x], [battery.v_dc])
    constraint.Ri = cs.Function("Ri", [x], [battery.Ri])

    constraint.machine_loss = cs.Function("machine_loss", [x, u], [machine.p_loss_total])

    constraint.inverter_loss = cs.Function("inverter_loss", [x, u], [inverter.p_loss_total])

    if pars_in["pwr"]["simple_loss"] is False:
        constraint.inverter_loss_switching = cs.Function("inverter_loss_switching", [x, u], [inverter.p_loss_switch])
        constraint.inverter_loss_conducting = cs.Function("inverter_loss_conducting", [x, u], [inverter.p_loss_cond])

    if pars_em["pwr"]["simple_loss"] is False:
        constraint.machine_loss_copper = cs.Function("machine_loss_copper", [x, u], [machine.p_loss_copper])
        constraint.machine_loss_statoriron = cs.Function("machine_loss_statoriron", [x, u],
                                                         [machine.p_loss_stator_iron])
        constraint.machine_loss_rotor = cs.Function("machine_loss_rotor", [x, u], [machine.p_loss_rotor])

    constraint.temp_cool_12 = cs.Function("temp_cool_12", [x], [radiators.temp_cool_12])
    constraint.temp_cool_13 = cs.Function("temp_cool_13", [x], [radiators.temp_cool_13])

    # ------------------------------------------------------------------------------------------------------------------
    # Model expressions ------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    f_impl = cs.vertcat(dx, dt * delta_t_alg_s)
    model.f_expl_expr_ipopt = f_impl
    model.f_impl_expr = f_impl - cs.vertcat(xdot, z)
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = cs.vertcat(kappa_disc_par)
    model.z = z

    return model, constraint
