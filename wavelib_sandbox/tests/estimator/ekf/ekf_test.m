addpath('../../../lib/util');
addpath('../../../lib/estimator/ekf');
addpath('../../../lib/model/omni_robot');

% simulation parameters
dt = 0.1;
mu = [0; 0; 0];
R = [
	0.05^2, 0, 0;
	0, 0.05^2, 0;
	0, 0, deg2rad(0.5)^2;
];
Q = [
	0.5^2, 0, 0;
	0, 0.5^2, 0;
	0, 0, deg2rad(10)^2;
];
S = eye(3);

% ekf
ekf = ekf_setup(dt, mu, R, Q, S);

% simulation parameters
t_end = 10;
T = 0:dt:t_end;
x_0 = [0; 0; 0];
x = zeros(length(x_0), length(T));
x(:, 1) = x_0;
y = zeros(length(ekf.Q), length(T));
u = [0.0; -10.0; 10.0];

% store
mu = zeros(length(x_0));
noise = zeros(length(ekf.R), length(x_0));


% simulation
for t = 2:length(T)
    % update state
    e = gaussian_noise(ekf.R);
    x(:, t) =  g(u, x(:, t - 1), dt) + e;
    noise(:, t) = e;

    % take measurement
    d = gaussian_noise(ekf.Q);
    y(:, t) = x(:, t) + d;

    % EKF
    [ekf] = ekf_prediction_update(ekf, @g, @G, u, 0);
    [ekf] = ekf_measurement_update(ekf, @h, @H, y(:, t), 0);

    % store ekf results
    mu(:, t) = ekf.mu;
end

plot_trajectory_2d(T, x, y, mu);