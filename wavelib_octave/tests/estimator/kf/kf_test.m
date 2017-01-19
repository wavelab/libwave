addpath('../../../lib/util');
addpath('../../../lib/estimator/kf');


% discrete time step
dt = 0.1;
mu = 10;  % mean (mu)
S = 1;  % covariance (Sigma)

% motion model
A = 0.8;
B = 3;
R = 2;

% measurement model
C = 1;
Q = 4;

% kalman filter
kf = kf_setup(mu, S, A, B, C, R, Q);

% Simulation Initializations
Tf = 3;
T = 0:dt:Tf;
x = zeros(1, length(T) + 1);
x(1) = mu + sqrt(S) * randn(1);
y = zeros(1, length(T));
u = zeros(1, length(T));
mup_S = zeros(length(mu), length(T));
mu_S = zeros(length(mu), length(T));



% main loop
for t = 1:length(T)
    % select control action
    if (t > 1)
        u(t) = u(t - 1);
    end
    if (mu > 10)
        u(t) = 0;
    elseif (mu < 2);
        u(t) = 1;
    end

    % select a motion disturbance
    e = sqrt(R) * randn(1);
    x(t + 1) = A * x(t) + B * u(t) + e;

    % take measurement
    d = sqrt(Q) * randn(1);
    y(t) = C * x(t + 1) + d;

    % kalman filter estimation
    kf = kf_estimate(kf, u(t), y(t));

    % store estimates
    mup_S(t)= kf.mu_p;
    mu_S(t)= kf.mu;
end
plot_trajectory_1d(T, x, y, mu_S)
