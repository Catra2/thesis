% This script configures simulink, initializes an MPC, and generates data
% as the controller sweeps through the entire (sequential) state-to-input
% space. 

% Should also be initialized from a diverse set of initial conditions

% triple mass-spring damper modeling (extracted from do-mpc)
c = [2.697 2.66 3.05 2.86]*1e-3
d = [6.78 8.01 8.82]*1e-5
Theta = (2.25*1e-4)
x0 = pi*[1, 1, -1.5, 1, -1, 1]
a41 = (-c(1)/Theta)+(-c(2)/Theta)
a42 = c(2)/Theta
a43 = 0
a44 = -(d(1)/Theta)
a45 = 0
a46 = 0

a51 = c(2)/Theta
a52 = -(c(2)/Theta) - (c(3)/Theta)
a53 = c(3)/Theta
a54 = 0
a55 = -d(2)/Theta
a56 = 0

a61 = 0
a62 = c(3)/Theta
a63 = -(c(3)/Theta) -(c(4)/Theta)
a64 = 0
a65 = 0
a66 = -d(3)/Theta

A = [
    0   0   0   1   0   0;
    0   0   0   0   1   0;
    0   0   0   0   0   1;
    a41 a42 a43 a44 a45 a46;
    a51 a52 a53 a54 a55 a56;
    a61 a62 a63 a64 a65 a66
    ]

B = [
    0 0;
    0 0;
    0 0;
    c(1)/Theta 0;
    0 0;
    0 c(4)/Theta
    ]

% x1, x2, x3 = angular position (+- from 'zero')
% x4, x5, x6 = angular rate
% Measured Output = MO = x1, x2, x3 (3 total) + (3 for plotting rate)
% All 6 states are observed because virtual sensors are ~~freeee~~!
C = eye(6,6)
D = zeros(6,2)
disc_model = ss(A,B,C,D)

% MPC

% Manipulated variable is the motor angle (in radians)
% Output
controller_ipopt = mpc(disc_model,0.1)
controller_ipopt.PredictionHorizon = 20;
controller_ipopt.ControlHorizon = 20;
controller_ipopt.Model.Nominal.U = [0;0];
controller_ipopt.Model.Nominal.Y = [0,0,0];
controller_ipopt.MV(1).Max = 2*pi;
controller_ipopt.MV(1).Min = -2*pi;
controller_ipopt.MV(2).Min = -2*pi;
controller_ipopt.MV(2).Max = 2*pi;
controller_ipopt.OV(1).Min = -2*pi;
controller_ipopt.OV(1).Max = 2*pi;
controller_ipopt.OV(2).Min = -2*pi;
controller_ipopt.OV(2).Max = 2*pi;
controller_ipopt.OV(3).Min = -2*pi;
controller_ipopt.OV(3).Max = 2*pi;
controller_ipopt.ManipulatedVariables(1).ScaleFactor = 2
controller_ipopt.ManipulatedVariables(2).ScaleFactor = 2

controller_ipopt.Weights.ManipulatedVariables = ones(1,2)*0.2
controller_ipopt.Weights.ManipulatedVariablesRate = ones(1,2)*0.05
controller_ipopt.Weights.OutputVariables = horzcat(ones(1,3),zeros(1,3))
controller_ipopt.Optimizer.Algorithm = 'interior-point'
controller_ipopt.Optimizer.UseSuboptimalSolution = false

% Test an alternative built-in optimizer
controller_other = mpc(disc_model,0.1)
controller_other.PredictionHorizon = 20;
controller_other.ControlHorizon = 20;
controller_other.Model.Nominal.U = [0;0];
controller_other.Model.Nominal.Y = [0,0,0];
controller_other.MV(1).Max = 2*pi;
controller_other.MV(1).Min = -2*pi;
controller_other.MV(2).Min = -2*pi;
controller_other.MV(2).Max = 2*pi;
controller_other.OV(1).Min = -2*pi;
controller_other.OV(1).Max = 2*pi;
controller_other.OV(2).Min = -2*pi;
controller_other.OV(2).Max = 2*pi;
controller_other.OV(3).Min = -2*pi;
controller_other.OV(3).Max = 2*pi;

% Set the relative weights in the cost function

controller_other.Weights.ManipulatedVariables = ones(1,2)*0.2

% Adding a small weight to the rate helps smooth out the MPC trajectory
controller_other.Weights.ManipulatedVariablesRate = ones(1,2)*0.01
controller_other.Weights.OutputVariables = horzcat(ones(1,3),zeros(1,3))

controller_other.Optimizer.Algorithm = 'active-set'


sim('data_generator_sim',5)