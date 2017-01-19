% Extended Kalman Filter Localization example
addpath('../../../lib/util');
addpath('../../../lib/plot');
addpath('../../../lib/sensor');
addpath('../../../lib/estimator/ekf');

% Time
Tf = 20;
dt = 0.1;
T = 0:dt:Tf;

% Initial State
x0 = [0; 0; 0];

% Prior
mu = [0; 0; 0]; % mean (mu)
S = 0.1 * eye(3);% covariance (Sigma)

% Control inputs
u = ones(2, length(T));
u(2, :) = 0.3 * u(2, :);

% Disturbance model
R = [1e-4 0 0;
     0 1e-4 0;
     0 0 1e-6];

% Measurement type and noise
meas = 2;  % 1 - range, 2 - bearing, 3 - both
data = struct('meas', meas);
switch(data.meas)
case 1
    Q = 0.01;
case 2
    Q = 0.01;
case 3
    Q = [
        0.01, 0;
        0, 0.01
    ];
end

% Feature Map
map = [
    5 5;
    3 1;
    -4 5;
    -2 3;
    0 4
];

% Simulation Initializations
n = length(x0);
x = zeros(n,length(T));
x(:, 1) = x0;
m = length(Q(:, 1));
y = zeros(m, length(T));
mup_S = zeros(n, length(T));
mu_S = zeros(n, length(T));
mf = zeros(2, length(T));

ekf = ekf_setup(dt, mu, R, Q, S);
ekf.meas = meas;
ekf.I = 1;



% main loop
for t = 2:length(T)
    % update state
    e = gaussian_noise(ekf.R);
    x(:, t) = g(u, x(:, t -1), dt, 0) + e;

    % take measurement
    d = gaussian_noise(ekf.Q);
    mf(:, t) = find_closest_feature(map, x(:, t));
    switch(data.meas)
    case 1  % 1 - range
        y(:, t) = max(0.001, range(mf(:, t), x(1:2, t)) + d);
    case 2  % 2 - bearing
        y(:, t) = bearing(mf(:, t), x(:, t));
    case 3  % 3 - both
        y(:,t) = [
            max(0.001, range(mf(:, t), x(1:2, t)));
            bearing(mf(:, t), x(:, t))
        ] + d;
    end

    % ekf
    [ekf] = ekf_prediction_update(ekf, @g, @G, u, 0);
    [ekf] = ekf_feature_measurement_update(ekf, @h, @H, mf(:, t), y(:, t), data);

    % store results
    mup_S(:,t) = ekf.mu_p;
    mu_S(:,t) = ekf.mu;
end


% Plot results
plot_results(T, map, mf, x, mu, S, mu_S, mup_S, y, meas);