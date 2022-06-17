"""
Run a simulation with the parameters specified in the configuration file

"""
from config import config
from utils.functions import timer, timeit, print_time
import numpy as np
import do_mpc
from casadi import *
import matplotlib.pyplot as plt
import matplotlib as mpl

# 2-motor, 3-disc, 4-spring, rotating inertial system with friction

# The model "block" has 5 properties:
# states, inputs, algebraic, parameter, timevarying_parameter
# states, inputs are required, the rest are optional
# for sysID technique and model, reference https://www.do-mpc.com/en/latest/getting_started.html

model = do_mpc.model.Model('continuous')

# three angular position states (phi), one for each disc
phi_1 = model.set_variable(var_type='states', var_name='phi_1', shape=(1,1))
phi_2 = model.set_variable(var_type='states', var_name='phi_2', shape=(1,1))
phi_3 = model.set_variable(var_type='states', var_name='phi_3', shape=(1,1))

# three angular velocities of each disc (dphi) as a vector
dphi = model.set_variable(var_type='states', var_name='dphi', shape=(3,1))

# two system inputs: set points
phi_m_1_set = model.set_variable(var_type='inputs', var_name='phi_m_1_set')
phi_m_2_set = model.set_variable(var_type='inputs', var_name='phi_m_2_set')

# two true motor positions
phi_1_m = model.set_variable(var_type='states', var_name='phi_1_m', shape=(1,1))
phi_2_m = model.set_variable(var_type='states', var_name='phi_2_m', shape=(1,1))

# confirm model contents
model.x.keys()

# model parameters
Theta_1 = model.set_variable('parameter', 'Theta_1')
Theta_2 = model.set_variable('parameter', 'Theta_2')
Theta_3 = model.set_variable('parameter', 'Theta_3')

# spring and friction coefficients are known constants
c = np.array([2.697, 2.66, 3.5, 2.86])*1e-3
d = np.array([6.78,  8.01, 8.82])*1e-5

# set the right-hand-side equation for 
model.set_rhs('phi_1', dphi[0])
model.set_rhs('phi_2', dphi[1])
model.set_rhs('phi_3', dphi[2])

# concatenate symbolic expressions for the state. CasADi is a symbolic matrix lib

dphi_next = vertcat(
    -c[0]/Theta_1*(phi_1-phi_1_m)-c[1]/Theta_1*(phi_1-phi_2)-d[0]/Theta_1*dphi[0],
    -c[1]/Theta_2*(phi_2-phi_1)-c[2]/Theta_2*(phi_2-phi_3)-d[1]/Theta_2*dphi[1],
    -c[2]/Theta_3*(phi_3-phi_2)-c[3]/Theta_3*(phi_3-phi_2_m)-d[2]/Theta_3*dphi[2],
)

# declare the right-hand side of the model ODE
model.set_rhs('dphi', dphi_next)
tau = 1e-2
model.set_rhs('phi_1_m', 1/tau*(phi_m_1_set - phi_1_m))
model.set_rhs('phi_2_m', 1/tau*(phi_m_2_set - phi_2_m))

# finalize the model (no further changes allowed)
model.setup()


# With the model defined, create the optimizer
mpc = do_mpc.controller.MPC(model)

# many parameters have default values, but we must set horizon and time step
setup_mpc = {
    'n_horizon': 20,
    't_step': 0.1,
    'n_robust': 1,
    'store_full_solution': True,
}
mpc.set_param(**setup_mpc)

mterm = phi_1**2 + phi_2**2 + phi_3**2
lterm = phi_1**2 + phi_2**2 + phi_3**2
mpc.set_objective(mterm, lterm)

# diagonal entries of the R matrix for penalizing control effort
mpc.set_rterm(
    phi_m_1_set = 1e-2,
    phi_m_2_set = 1e-2
)

# constraints

mpc.bounds['lower', '_x', 'phi_1'] = -2*np.pi
mpc.bounds['lower', '_x', 'phi_2'] = -2*np.pi
mpc.bounds['lower', '_x', 'phi_3'] = -2*np.pi

mpc.bounds['upper', '_x', 'phi_1'] = 2*np.pi
mpc.bounds['upper', '_x', 'phi_2'] = 2*np.pi
mpc.bounds['upper', '_x', 'phi_3'] = 2*np.pi

mpc.bounds['lower', '_u', 'phi_m_1_set'] = -2*np.pi
mpc.bounds['lower', '_u', 'phi_m_2_set'] = -2*np.pi

mpc.bounds['upper', '_u', 'phi_m_1_set'] = 2*np.pi
mpc.bounds['upper', '_u', 'phi_m_2_set'] = 2*np.pi

# scaling (can be skipped if states have similar magnitudes)
# mpc.scaling['_x', 'phi_1'] = 2
# etc

# uncertain parameters
inertia_mass_1 = 2.25*1e-4*np.array([1.0, 0.9, 1.1])
inertia_mass_2 = 2.25*1e-4*np.array([1.0, 0.9, 1.1])
inertia_mass_3 = 2.25*1e-4*np.array([1.0])

mpc.set_uncertainty_values(
    Theta_1 = inertia_mass_1,
    Theta_2 = inertia_mass_2,
    Theta_3 = inertia_mass_3
)

# finalize the optimizer (no further changes allowed)
mpc.setup()

# simulator
simulator = do_mpc.simulator.Simulator(model)

# there are many parameter options for the simulator, most with default values
simulator.set_param(t_step = 0.1)

# provide a function for the simulator to request the "true" values of
# uncertain parameters defined in the model (inertia of the masses, here)
p_template = simulator.get_p_template()

# the indexable keys are:
p_template.keys()
# output: ['default', 'Theta_1', 'Theta_2', 'Theta_3']

def p_fun(t_now):
  p_template['Theta_1'] = 2.25e-4
  p_template['Theta_2'] = 2.25e-4
  p_template['Theta_3'] = 2.25e-4
  return p_template

simulator.set_p_fun(p_fun)

# finalize simulator
simulator.setup()

# initial state (example)
x0 = np.pi*np.array([1, 1, -1.5, 1, -1, 1, 0, 0]).reshape(-1,1)

# assign the initial state to the various objects
simulator.x0 = x0
mpc.x0 = x0
mpc.set_initial_guess()

# vizualisation is supported in do_mpc as the "graphics" module
# information about each object is stored in object_name.data

# formatting options
mpl.rcParams['font.size'] = 18
mpl.rcParams['lines.linewidth'] = 3
mpl.rcParams['axes.grid'] = True

mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
sim_graphics = do_mpc.graphics.Graphics(simulator.data)

# create plot but don't show it yet
fig, ax = plt.subplots(2, sharex=True, figsize=(16,9))
fig.align_ylabels()

for g in [sim_graphics, mpc_graphics]:
  g.add_line(var_type='_x', var_name='phi_1', axis=ax[0])
  g.add_line(var_type='_x', var_name='phi_2', axis=ax[0])
  g.add_line(var_type='_x', var_name='phi_3', axis=ax[0])

  g.add_line(var_type='_u', var_name='phi_m_1_set', axis=ax[1])
  g.add_line(var_type='_u', var_name='phi_m_2_set', axis=ax[1])

ax[0].set_ylabel('position angle [rad]')
ax[1].set_ylabel('motor angle [rad]')
ax[1].set_xlabel('time [s]')

# uncontrolled response from non-zero init. state
u0 = np.zeros((2,1))
tic = timer()
for i in range(200):
  simulator.make_step(u0)
  # control law would insert a line here similar to: u0 = mpc.make_step(simulator.make_step(u0)) in a loop above
toc = timer()
print_time(tic,toc)
# 
sim_graphics.plot_results()
sim_graphics.reset_axes()
fig

# controlled response from non-zero init. state
sim_graphics.clear()

# main method during runtime. Called at each timestep and returns the control
#   input for the current initial state x0. Calls solve() and updates do_mpc.data.Data
tic = timer()
u0 = mpc.make_step(x0)
toc = timer()
print_time(tic,toc)

mpc_graphics.plot_predictions()
mpc_graphics.reset_axes()
fig

# Get ready to plot the full control loop graphics (cosmetic)

# Change the color for the three states:
for line_i in mpc_graphics.pred_lines['_x', 'phi_1']: line_i.set_color('#1f77b4') # blue
for line_i in mpc_graphics.pred_lines['_x', 'phi_2']: line_i.set_color('#ff7f0e') # orange
for line_i in mpc_graphics.pred_lines['_x', 'phi_3']: line_i.set_color('#2ca02c') # green
# Change the color for the two inputs:
for line_i in mpc_graphics.pred_lines['_u', 'phi_m_1_set']: line_i.set_color('#1f77b4')
for line_i in mpc_graphics.pred_lines['_u', 'phi_m_2_set']: line_i.set_color('#ff7f0e')

# Make all predictions transparent:
for line_i in mpc_graphics.pred_lines.full: line_i.set_alpha(0.2)

# Get line objects (note sum of lists creates a concatenated list)
lines = sim_graphics.result_lines['_x', 'phi_1']+sim_graphics.result_lines['_x', 'phi_2']+sim_graphics.result_lines['_x', 'phi_3']

ax[0].legend(lines,'123',title='disc')

# also set legend for second subplot:
lines = sim_graphics.result_lines['_u', 'phi_m_1_set']+sim_graphics.result_lines['_u', 'phi_m_2_set']
ax[1].legend(lines,'12',title='motor')

# Full control loop using all previously displayed "pieces"
simulator.reset_history()
simulator.x0 = x0
mpc.reset_history()
tic = timer()
for i in range(20):
  u0 = mpc.make_step(x0)
  x0 = simulator.make_step(u0)
  # note: timer showed ~ 450 ms for one iteration (time to beat with LSTM!)

toc = timer()
print_time(tic,toc)

# Plot the predictions for each uncertain parameter (inertia in this case)
mpc_graphics.plot_predictions(t_ind=0)
sim_graphics.plot_results()
sim_graphics.reset_axes()
fig