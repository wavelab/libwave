addpath('plot');
addpath('../../../lib/control');


%% Linear Quadratic Regulator - Pitch Control of Aircraft
clc; 
clear;

%% Simulation setup
t0 = 0;
tf = 100;
dt = 0.1;
T = t0:dt:tf;

%% Continuous time model

% State x = [angle of attack, pitch, pitch rate]
A = [-0.313, 0 56.7; 0,  0, 56.7; -0.0139, 0, -0.426];
B = [0.232; 0; 0.0203];
C = [0, 1, 0];
D = [0];

sys = ss(A, B, C, D);

%% Discretized model
sysd = c2d(sys, dt, 'zoh');
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

x0 = [1, 0, 0];

% LQR costs
Q = 0.01 * eye(3);  % state cost
R = 0.01 * eye(1);  % input effort cost (small R cost more, large R cost less)

[x, u, Jx, Ju, P, K] = lqr_sim(Ad, Bd, Q, R, t0, tf, dt, x0, 0);
[xss, uss, Jxss, Juss, Pss, Kss] = lqr_sim(Ad, Bd, Q, R, t0, tf, dt, x0, 1);
[x2, u2, Jx2, Ju2, P2, K2] = lqr_sim(Ad, Bd, Q, 10 * R, t0, tf, dt, x0, 0);
[x3, u3, Jx3, Ju3, P3, K3] = lqr_sim(Ad, Bd, Q, 100 * R, t0, tf, dt, x0, 0);

% % Default run plot
% plot_states(1, T, x, u);
%  
% % Plots for steady state
% plot_steady_state(2, T, x, xss, u, uss);
% plot_costate(3, P, Q);

% Plots for Q,R comparison
plot_comparison(4, T, x, x2, x3, u, u2, u3, Ju, Jx, Ju2, Jx2, Ju3, Jx3);
