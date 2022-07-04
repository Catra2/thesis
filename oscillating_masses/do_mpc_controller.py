"""
Run a simulation with the parameters specified in the do-mpc tutorial.
The system to be simulated is a 2-motor, 3-disc, 4-spring, rotating inertial mass with friction.
Work in progress 6/27/2022.

"""
import numpy as np
import do_mpc
from casadi import *
import matplotlib.pyplot as plt
import matplotlib as mpl
from utils.functions import timer, timeit

# The model "block" has 5 properties:
# states, inputs, algebraic, parameter, timevarying_parameter
# states, inputs are required, the rest are optional

model = do_mpc.model.Model('continuous')

# Three angular position states (phi), one for each disc
phi = model.set_variable(var_type='_x', var_name='phi', shape=(3, 1))
dphi = model.set_variable(var_type='_x', var_name='dphi', shape=(3, 1))

# Two states for the desired (set) motor position:
phi_m_set = model.set_variable(var_type='_u', var_name='phi_m_set', shape=(2, 1))

# Two additional states for the true motor position:
phi_m = model.set_variable(var_type='_x', var_name='phi_m', shape=(2, 1))

# State measurements
phi_meas = model.set_meas('phi_meas', phi, meas_noise=True)

# Input measurements
phi_m_set_meas = model.set_meas('phi_m_set_meas', phi_m_set, meas_noise=False)

# Confirm model contents by calling the keys
model.x.keys()

# Model parameters
Theta_1 = model.set_variable('parameter', 'Theta_1')
Theta_2 = model.set_variable('parameter', 'Theta_2')
Theta_3 = model.set_variable('parameter', 'Theta_3')

# Spring and friction coefficients are known constants
c = np.array([2.697, 2.66, 3.5, 2.86]) * 1e-3
d = np.array([6.78, 8.01, 8.82]) * 1e-5

model.set_rhs('phi', dphi)

dphi_next = vertcat(
    -c[0] / Theta_1 * (phi[0] - phi_m[0]) - c[1] / Theta_1 * (phi[0] - phi[1]) - d[0] / Theta_1 * dphi[0],
    -c[1] / Theta_2 * (phi[1] - phi[0]) - c[2] / Theta_2 * (phi[1] - phi[2]) - d[1] / Theta_2 * dphi[1],
    -c[2] / Theta_3 * (phi[2] - phi[1]) - c[3] / Theta_3 * (phi[2] - phi_m[1]) - d[2] / Theta_3 * dphi[2],
)

model.set_rhs('dphi', dphi_next, process_noise=False)
tau = 1e-2
model.set_rhs('phi_m', 1 / tau * (phi_m_set - phi_m))

# Finalize the model (no further changes allowed after this)
model.setup()
model.x.labels
# With the model defined, create the optimizer
mpc = do_mpc.controller.MPC(model)

# Many parameters have default values, but we must set horizon and time step
setup_mpc = {
    'n_horizon': 20,
    't_step': 0.1,
    'n_robust': 1,
    'store_full_solution': True,
}
mpc.set_param(**setup_mpc)

# At the heart of MPC is a user-defined cost function (also called an objective function). The following lines are the
# interface for the engineer to tell the optimizer, "The cost of the current state is the total disarray in the system"
mterm = phi[0] ** 2 + phi[1] ** 2 + phi[2] ** 2
lterm = phi[0] ** 2 + phi[1] ** 2 + phi[2] ** 2
mpc.set_objective(mterm, lterm)

# In addition to there being a 'cost' to the disarray in the system, there is also a cost for the amount of control
# effort used (fuel or electricity spent to manipulate a motor, for example). Diagonal entries of the R matrix are used
# to penalize this control effort, providing a balance between system disorder and control effort.
mpc.set_rterm(
    phi_m_set=1e-2
)

# Constraints are an important part of why MPC is desirable. They also make the optimization process more difficult.
mpc.bounds['lower', '_x', 'phi'] = -2 * np.pi
mpc.bounds['upper', '_x', 'phi'] = 2 * np.pi

mpc.bounds['lower', '_u', 'phi_m_set'] = -2 * np.pi
mpc.bounds['upper', '_u', 'phi_m_set'] = 2 * np.pi

# Uncertain parameters are also passed to the controller so the optimizer can calculate a path for each given scenario.
inertia_mass_1 = 2.25 * 1e-4 * np.array([1.0, 0.9, 1.1])
inertia_mass_2 = 2.25 * 1e-4 * np.array([1.0, 0.9, 1.1])
inertia_mass_3 = 2.25 * 1e-4 * np.array([1.0])

mpc.set_uncertainty_values(
    Theta_1=inertia_mass_1,
    Theta_2=inertia_mass_2,
    Theta_3=inertia_mass_3
)

# Finalize the optimizer (no further changes allowed)
mpc.setup()

# Define the model to be used in simulation
simulator = do_mpc.simulator.Simulator(model)

# There are *many* parameter options for the simulator, most with default values, but we will set the time_step
simulator.set_param(t_step=0.1)

# Provide a function for the simulator to request the "true" values of
# uncertain parameters defined in the model (inertia of the masses, in this case)
p_template = simulator.get_p_template()


# Indexed keys can be viewed to confirm an intended configuration
# p_template.keys()  # output: ['default', 'Theta_1', 'Theta_2', 'Theta_3']


# The simulator will use this function to run 'worst-case' scenarios based on the uncertainty in our measurements,
# which we can define here. These indicate the dashed lines you see in the trajectory visualization.
def p_fun(t_now):
    p_template['Theta_1'] = 2.25e-4
    p_template['Theta_2'] = 2.25e-4
    p_template['Theta_3'] = 2.25e-4
    return p_template


simulator.set_p_fun(p_fun)

# Finalize simulator, restricting any further changes to parameters within it
simulator.setup()

# Since we will use a sequential input (LSTM), the MPC equivalent state estimator will be an MHE
mhe = do_mpc.estimator.MHE(model, ['Theta_1'])

setup_mhe = {
    'n_horizon': 10,
    't_step': 0.1,
    'store_full_solution': True,
    'meas_from_data': True
}
mhe.set_param(**setup_mhe)

P_v = np.diag(np.array([1, 1, 1]))
P_x = np.eye(8)
P_p = 10 * np.eye(1)

mhe.set_default_objective(P_x, P_v, P_p)
p_template_mhe = mhe.get_p_template()


def p_fun_mhe(t_now):
    p_template_mhe['Theta_2'] = 2.25e-4
    p_template_mhe['Theta_3'] = 2.25e-4
    return p_template_mhe


mhe.set_p_fun(p_fun_mhe)

mhe.bounds['lower', '_u', 'phi_m_set'] = -2 * np.pi
mhe.bounds['upper', '_u', 'phi_m_set'] = 2 * np.pi

mhe.bounds['lower', '_p_est', 'Theta_1'] = 1e-5
mhe.bounds['upper', '_p_est', 'Theta_1'] = 1e-3

mhe.setup()

# Set the initial state, which we will initialize to be (randomly) non-zero for this example
x0 = np.pi * np.array([1, 1, -1.5, 1, -1, 1, 0, 0]).reshape(-1, 1)
x0_mhe = x0 * (1 + 0.5 * np.random.randn(8, 1))  # Give the estimator a bad initial state for fun

# Each object needs to know what the initial state is
simulator.x0 = x0
mhe.x0_mhe = x0_mhe
mhe.p_est0 = 1e-4
mhe.set_initial_guess()
mpc.x0 = x0
mpc.set_initial_guess()

# Visualization is supported in do_mpc as the "graphics" module, information about each object is stored in
# object_name.data

# Set some formatting options to make the graphs uniquely distinguishable
mpl.rcParams['font.size'] = 18
mpl.rcParams['lines.linewidth'] = 3
mpl.rcParams['axes.grid'] = True
mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
sim_graphics = do_mpc.graphics.Graphics(simulator.data)

# Create a plot but don't show it (yet)
fig, ax = plt.subplots(2, sharex=True, figsize=(16, 9))
fig.align_ylabels()

for g in [sim_graphics, mpc_graphics]:
    g.add_line(var_type='_x', var_name='phi_1', axis=ax[0])

    g.add_line(var_type='_u', var_name='phi_m_set', axis=ax[1])

ax[0].set_ylabel('position angle [rad]')
ax[1].set_ylabel('motor angle [rad]')
ax[1].set_xlabel('time [s]')

# Simulate an uncontrolled response from a non-zero initial state. This will show that the system is indeed sub-optimal
# without the controller, and act as a baseline.
u0 = np.zeros((2, 1))
for i in range(200):
    simulator.make_step(u0)
    # Note: control law would insert a line here similar to: u0 = mpc.make_step(simulator.make_step(u0)) in a loop

sim_graphics.plot_results()
sim_graphics.reset_axes()
fig
plt.show()

# Now let's view the controlled response after resetting the simulation.
sim_graphics.clear()

# Take a single step, as an example for this code.
u0 = mpc.make_step(x0)

mpc_graphics.plot_predictions()
mpc_graphics.reset_axes()
fig
plt.show()

# Get ready to plot the full control loop graphics (cosmetic):

# Change the color for the three states:
for line_i in mpc_graphics.pred_lines['_x', 'phi', 0]: line_i.set_color('#1f77b4')  # blue
for line_i in mpc_graphics.pred_lines['_x', 'phi', 1]: line_i.set_color('#ff7f0e')  # orange
for line_i in mpc_graphics.pred_lines['_x', 'phi', 2]: line_i.set_color('#2ca02c')  # green

# Change the color for the two inputs:
for line_i in mpc_graphics.pred_lines['_u', 'phi_m_set', 0]: line_i.set_color('#1f77b4')  # blue
for line_i in mpc_graphics.pred_lines['_u', 'phi_m_set', 1]: line_i.set_color('#ff7f0e')  # orange

# Make all predictions transparent:
for line_i in mpc_graphics.pred_lines.full: line_i.set_alpha(0.2)

# Get line objects (note sum of lists creates a concatenated list)
lines = sim_graphics.result_lines['_x', 'phi', 0] \
        + sim_graphics.result_lines['_x', 'phi', 1] \
        + sim_graphics.result_lines['_x', 'phi', 2]

ax[0].legend(lines, '123', title='disc')

# Also set legend for second subplot:
lines = sim_graphics.result_lines['_u', 'phi_m_set', 1] + sim_graphics.result_lines['_u', 'phi_m_set', 2]
ax[1].legend(lines, '12', title='motor')

# Full control loop using all previously configured "pieces" to this puzzle
simulator.reset_history()
simulator.x0 = x0
mpc.reset_history()
mhe.reset_history()
mhe.x0 = x0_mhe

time_mpc = []
time_mhe = []
for i in range(20):
    tic = timer()
    u0 = mpc.make_step(x0)
    toc = timer()
    time_mpc.append(timeit(tic, toc))
    # Note: timer showed mean 190 ~ 244 [ms] per iteration for MPC

    y_next = simulator.make_step(u0)

    tic = timer()
    x0 = mhe.make_step(y_next)
    toc = timer()
    time_mhe.append(timeit(tic, toc))
    # Note: timer showed mean 28 [ms] per iteration for MHE

    # The simulator is not included in the timing because it will not be a part of the control loop

avg_mpc = np.mean(time_mpc)
avg_mhe = np.mean(time_mhe)
print(f"{avg_mpc : .3f}, {avg_mhe : .3f}")
# Plot the predictions for each uncertain parameter (inertia in this case)
mpc_graphics.plot_predictions(t_ind=0)
sim_graphics.plot_results()
sim_graphics.reset_axes()
fig
plt.show()
# %%
