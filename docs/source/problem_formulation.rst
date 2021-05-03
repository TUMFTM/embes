.. _refProblemFormulation:

Problem formulation
===================

Basically, the energy strategy works by solving an Optimal Control Problem. Its objective function is the lethargy, i
.e., the time the vehicle takes to complete the race,

    .. math::
        J(x(s)) = \int_{0}^{s_\mathrm{f}}{\frac{1}{v(s)}\mathrm{d}s},

where :math:`v(s)` is the velocity and :math:`s` the independent distance variable.

The state variables in the optimization problem are:

    .. math::
        \boldsymbol{x}(s) = \left(v~t~\sigma~T_\mathrm{B}~T_\mathrm{M}~T_\mathrm{I}~T_\mathrm{CM}~T_\mathrm{CB
        }\right)^T \in \mathbb{R}^{n_x \times 1},

where :math:`t(s)` describes the time, :math:`\sigma(s)` the battery SOC, and :math:`T_\mathrm{B}`,
:math:`T_\mathrm{M}`, :math:`T_\mathrm{I}`, :math:`T_\mathrm{CM}`, :math:`T_\mathrm{CB}` the temepratures of battery,
electric machines, inverters, colling liquid machine-inverter and cooling liquid battery.

For convenience, we also included :math:`s(s)` in the state vector to simplify plotting the results.
The control vector contains

    .. math::
        \boldsymbol{u}(s) = \left(F_\mathrm{d}~F_\mathrm{b}\right)^T \in \mathbb{R}^{n_u \times 1},

where :math:`F_\mathrm{d}` is the drive and :math:`F_\mathrm{b}` the brake force.

The algebraic variable

    .. math::
        z(s) = \frac{1}{v} \in \mathbb{R}^{n_z \times 1}

describes the time spent between two spatial discretization points and depicts the main contribution to the objective
function.

The detailed mathematical problem formulation behind the code and further explanations can be found in our publication.
The preprint of this publication can be found on arXiv:

The paper will soon be available on SAE Mobilus:

Convergence
===========

If the solver does not convergence in the *presolve* or the *resolve*
step, it is likely that the optimization horizon is too short. The energy strategy makes only sense if the brake
force gets restricted by nature. This means that simultaneous activation of the drive and brake force must be avoided
by a sufficient length of the optimization horizon. **If the race is too short, and no energetic or thermodynamic
constraint becomes active, no energy strategy is necessary**. A full speed operation is then appropriate.
