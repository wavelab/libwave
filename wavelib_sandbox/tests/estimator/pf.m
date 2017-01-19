source('lib/estimator/pf.m');
source('lib/estimator/kf.m');


% Particle filter example
clear;
clc;

% Discrete time step
dt = 0.1;

% Prior
mu = 10; % mean (mu)
S = 1;% covariance (Sigma)

% Number of particles
M = 10;
% Prior - uniform over 5 15
X = 5+10*rand(1,M);
X0 = X;

%Motion model
A = 0.8;
B = 3;
R = 1;

% Measurement model
C = 1;
Q = 1;

% Simulation Initializations
Tf = 3;
T = 0:dt:Tf;
x = zeros(1, length(T) + 1);
x(1) = mu + sqrt(S) * randn(1);
y = zeros(1, length(T));
u = zeros(1, length(T));

% estimators
pf = PF(dt, M, R, Q);
kf = KF(mu, S, A, B, C, R, Q);


function plot_trajectory_results(figure_index, T, x, y, u, mu_S, muP_S)
    figure(1);
    clf;
    hold on;

    % plots
    plot(T, x(2:end), 'b');  % state
    plot(T, y, 'ro');  % measurement
    plot(T, u, 'g--');  % input
    plot(T, mu_S,'r--');  % kf belief
    plot(T, muP_S, 'co--');  % pf belief

    % plot details
    title('State and Estimates')
    try
        legend(
            'State',
            'Measurement',
            'Input',
            'Kalman Estimate',
            'Particle Estimate'
        )
    catch
    end_try_catch

    pause;
end

function Xp_m = Xp(X, u, R)
    A = 0.8;
    B = 3;

    e = sqrt(R) * randn(1);
    Xp_m = A * X + B * u + e;
end

function hXp_m = hXp(Xp_m)
    C = 1;

    hXp_m = C * Xp_m;
end


% main loop
tic;
for t = 1:length(T)
    % control action
    if (t > 1)
        u(t) = u(t - 1);
    end
    if (mu > 10)
        u(t) = 0;
    elseif (mu < 2);
        u(t) = 1;
    end

    % update state
    e = sqrt(R) * randn(1);
    x(t + 1) = A * x(t) + B * u(t) + e;

    % take measurement
    d = sqrt(Q) * randn(1);
    y(t) = C * x(t + 1) + d;

    % kalman filter estimation
    kf = kf_estimate(kf, u(t), y(t));
    mup_S(t)= kf.mu_p;
    mu_S(t)= kf.mu;

    % Particle filter estimation
    pf = pf_estimate(pf, @Xp, @hXp, u(t), y(t));
    muP_S(t) = pf.mu;
    SP_S(t) = pf.S;
end
toc;

plot_trajectory_results(1, T, x, y, u, mu_S, muP_S);
