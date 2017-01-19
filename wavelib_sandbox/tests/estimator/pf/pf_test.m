addpath('../../../lib/estimator/pf');
addpath('../../../lib/estimator/kf');
addpath('../../../lib/util/');


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
X = 5 + 10 * rand(1, M);
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
pf = pf_setup(dt, M, R, Q);
kf = kf_setup(mu, S, A, B, C, R, Q);

kf_mu_p_store = zeros(length(kf.mu_p), length(T));
kf_mu_store = zeros(length(kf.mu), length(T));
pf_mu_p_store = zeros(length(pf.mu), length(T));
pf_S_store = zeros(length(pf.S), length(T));


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
    kf_mu_p_store(t)= kf.mu_p;
    kf_mu_store(t)= kf.mu;

    % Particle filter estimation
    pf = pf_estimate(pf, @Xp, @hXp, u(t), y(t));
    pf_mu_p_store(t) = pf.mu;
    pf_S_store(t) = pf.S;
end
toc;

plot_results(1, T, x, y, u, kf_mu_store, pf_mu_p_store);
pause
