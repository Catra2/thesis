% This script configures simulink, initializes an MPC, and generates data
% as the controller sweeps through the entire (sequential) state-to-input
% space. 

% triple mass-spring damper modeling (extracted from do-mpc)
c = [2.697 2.66 3.05 2.86]*1e-3
d = [6.78 8.01 8.82]*1e-5
Theta = [1 1 1]*2.25*1e-4
a41 = -(c(1)/Theta(1))-(c(2)/Theta(1))
a42 = c(2)/Theta(1)
a43 = 0
a44 = -(d(1)/Theta(1))
a45 = 0
a46 = 0

a51 = c(2)/Theta(2)
a52 = -(c(2)/Theta(2)) - (c(3)/Theta(2))
a53 = c(3)/Theta(2)
a54 = 0
a55 = -d(2)/Theta(2)
a56 = 0

a61 = 0
a62 = c(3)/Theta(3)
a63 = -c(3)/Theta(3)
a64 = -c(4)/Theta(3)
a65 = 0
a66 = -d(3)/Theta(3)

A = [
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    a41 a42 a43 a44 a45 a46;
    a51 a52 a53 a54 a55 a56;
    a61 a62 a63 a64 a65 a66
    ]

B = [
    0 0;
    0 0;
    0 0;
    c(1)/Theta(1) 0;
    0 0;
    0 c(4)/Theta(3)
    ]

C = [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0]
D = [0,0;0,0;0,0]
disc_model = ss(A,B,C,D)

% MPC

% Manipulated variable is the motor angle (in radians)
% Output
controller = mpc(disc_model,0.1)
controller.PredictionHorizon = 20;
controller.ControlHorizon = 2;
controller.Model.Nominal.U = [0;0];
controller.Model.Nominal.Y = [0, 0, 0];
controller.MV(1).Max = 2*pi;
controller.MV(1).Min = -2*pi;
controller.MV(2).Min = -2*pi;
controller.MV(2).Max = 2*pi;
controller.OV(1).Min = -2*pi;
controller.OV(1).Max = 2*pi;







