% This script configures simulink, initializes an MPC, and generates data
% as the controller sweeps through the entire (sequential) state-to-input
% space. 

% Should also be initialized from a diverse set of initial conditions

% triple mass-spring damper modeling (extracted from do-mpc)
c = [2.697 2.66 3.05 2.86]*1e-3;
d = [6.78 8.01 8.82]*1e-5;
Theta = (2.25*1e-4);
% x0 = pi*[1, 1, -1.5, 1, -1, 1];
a41 = (-c(1)/Theta)+(-c(2)/Theta);
a42 = c(2)/Theta;
a43 = 0;
a44 = -(d(1)/Theta);
a45 = 0;
a46 = 0;

a51 = c(2)/Theta;
a52 = -(c(2)/Theta) - (c(3)/Theta);
a53 = c(3)/Theta;
a54 = 0;
a55 = -d(2)/Theta;
a56 = 0;

a61 = 0;
a62 = c(3)/Theta;
a63 = -(c(3)/Theta) -(c(4)/Theta);
a64 = 0;
a65 = 0;
a66 = -d(3)/Theta;

A = [
    0   0   0   1   0   0;
    0   0   0   0   1   0;
    0   0   0   0   0   1;
    a41 a42 a43 a44 a45 a46;
    a51 a52 a53 a54 a55 a56;
    a61 a62 a63 a64 a65 a66
    ];

B = [
    0 0;
    0 0;
    0 0;
    c(1)/Theta 0;
    0 0;
    0 c(4)/Theta
    ];

% x1, x2, x3 = angular position (+- from 'zero')
% x4, x5, x6 = angular rate
% Measured Output = MO = x1, x2, x3 (3 total) + (3 for plotting rate)
% All 6 states are observed because virtual sensors are ~~freeee~~!
C = eye(6,6);
D = zeros(6,2);
disc_model = ss(A,B,C,D);

% MPC
horizon = 20; % steps ahead
tstep = 0.1; % seconds
simTime = 10; % seconds

% Manipulated variable is the motor angle (in radians)
% Output
controller = mpc(disc_model,tstep);
controller.PredictionHorizon = 20;
controller.ControlHorizon = 20;
controller.Model.Nominal.U = [0;0];
controller.Model.Nominal.Y = [0,0,0];
controller.MV(1).Max = 2*pi;
controller.MV(1).Min = -2*pi;
controller.MV(2).Min = -2*pi;
controller.MV(2).Max = 2*pi;
controller.OV(1).Min = -2*pi;
controller.OV(1).Max = 2*pi;
controller.OV(2).Min = -2*pi;
controller.OV(2).Max = 2*pi;
controller.OV(3).Min = -2*pi;
controller.OV(3).Max = 2*pi;
controller.ManipulatedVariables(1).ScaleFactor = 2;
controller.ManipulatedVariables(2).ScaleFactor = 2;

controller.Weights.ManipulatedVariables = ones(1,2)*0.2;
controller.Weights.ManipulatedVariablesRate = ones(1,2)*0.01; % smoother
controller.Weights.OutputVariables = horzcat(ones(1,3),zeros(1,3));

controller.Optimizer.Algorithm = 'active-set';
controller.Optimizer.UseSuboptimalSolution = false;
%% Main loop to run a simulation
% 
% N = 100;
% x0 = zeros(N,6); % pre-allocation 
% 
% for i = 1:N
%     % initial states to explore
%     x0(i,:) = -2*pi + (4*pi)*rand(1,6);
%     out = sim('data_generator_sim'); % generate 10 seconds of response data
%     
%     % Viewing and exporting data
%     move = out.move;
%     response = out.response;
%     reference = zeros(1,6); % only the first three have controller.Weights
% 
%     writematrix(response,'response.txt','WriteMode','append') % (101*N)x6
%     writematrix(move,'move.txt','WriteMode','append') % 101x2 per
%     writematrix(reference,'reference.txt','WriteMode','append') % 1x6 per
% end    