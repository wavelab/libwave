addpath('../../../lib/util');
addpath('../../../lib/plot');
addpath('../../../lib/sensor');
addpath('../../../lib/estimator/ekf');
addpath('plot');


% Time
Tf = 40;
dt = 0.5;
T = 0:dt:Tf;

% Initial Robot State
x0 = [0 0 0]';

% Control inputs
u = ones(2, length(T));
u(2,:) = 0.3 * u(2, :);

% Motion Disturbance model
R = [0.001, 0, 0;
     0, 0.001, 0;
     0, 0, 0.001];

% Prior over robot state
mu0r = [0; 0; 0];  % mean (mu)
S0rr = 0.00000000001 * eye(3);  % covariance (Sigma)

% Feature Map
map = [
    -5:1:5, 5:-1:-5; 
    -2 * ones(1, 11), 8 * ones(1, 11)
];
M = length(map);

% Prior over feature map
mu0m = zeros(2 * M, 1);
S0mm = 100 * eye(2 * M);
newfeature = ones(M, 1);

% Measurement model
rmax = 5;  % Max range
thmax = pi / 4;  % 1/2 Field of view

% Measurement noise
Qi = [
    0.00001, 0; 
    0, 0.00001
];

% Simulation Initializations
n = length(R(:, 1));  % Number of vehicle states
xr = zeros(n, length(T));  % Vehicle states 
xr(:, 1) = x0;
N = n + 2 * M;
m = length(Qi(:, 1)); % Number of measurements per feature 
y = zeros(m * M, length(T)); % Measurements

mu = [mu0r; mu0m];  % belief of robot state and feature state
S = [S0rr zeros(n, 2 * M);  zeros(2 * M, n) S0mm];
%S = [S0rr 100*ones(n,2*M);  100*ones(2*M,n) S0mm];

mu_S = zeros(N,length(T)); % Belief
mu_S(:,1) = mu;


%% plot first time step
t = 1;
[newfeature] = plot_step(1, t, map, xr, mu, mu_S, S, y, newfeature);

% main loop
for t = 2:length(T)
    % Update robot state
    e = gaussian_noise(R);
    xr(:,t) = [
        xr(1, t - 1) + u(1, t) * cos(xr(3, t - 1)) * dt;
        xr(2, t - 1) + u(1, t) * sin(xr(3, t - 1)) * dt;
        xr(3, t - 1) + u(2, t) * dt ...
    ] + e;

    % Take measurements - for each feature
    flist = zeros(M, 1);  % list to track which features are detected
    for i = 1:M
        if (is_feature_inview(map(:, i), xr(:, t), rmax, thmax))
            flist(i) = 1;
            
            % Determine measurement
            d = gaussian_noise(Qi);
            y((2 * (i - 1) + 1):(2 * i), t) = [ ...
                range(map(:, i), xr(:, t));
                bearing(map(:, i), xr(:, t))
            ] + d;
        end
    end
    
    %% Extended Kalman Filter Estimation
    % Prediction update
    mu(1:3) = [
        mu(1) + u(1 ,t) * cos(mu(3)) * dt;
        mu(2) + u(1, t) * sin(mu(3)) * dt;
        mu(3) + u(2, t) * dt
    ];
    
    Gt = [ 
        1, 0, -u(1, t) * sin(mu(3)) * dt;
        0, 1, u(1, t) * cos(mu(3)) * dt;
        0, 0, 1
    ];
    
    S(1:n, 1:n) = Gt * S(1:n, 1:n) * Gt' + R;

    % Measurement update
    for i = 1:M
        if (flist(i))
            % index of mu4 and mu5 (measurement x and y)
            mu4 = 3 + 2 * (i - 1) + 1;
            mu5 = 3 + 2 * i;
            
            % Feature initialization
            if (newfeature(i) == 1)
                mu(mu4) = mu(1) + y(2 * (i - 1) + 1, t) * cos(y(2 * i, t) + mu(3));
                mu(mu5) = mu(2) + y(2 * (i - 1) + 1, t) * sin(y(2 * i, t) + mu(3));
                newfeature(i) = 0;
            end

            % Linearization
            % predicted range
            dx = mu(mu4) - mu(1);
            dy = mu(mu5) - mu(2);
            rp = sqrt((dx)^2 + (dy)^2);

            % feature selector vector
            Fi = zeros(5, N); % N is nb of elements (nb vehicle states + 2 * nb features)
            Fi(1:n, 1:n) = eye(n);  % robot x, y, theta
            Fi(4:5, mu4:mu5) = eye(2);  % measurement x, y
            
            % linearization of measurement
            Ht = [ ...
                (-dx / rp), (-dy / rp), (0), (dx / rp), (dy / rp);  % range linearization
                (dy / rp^2), (-dx / rp^2), (-1), (-dy / rp^2), (dx / rp^2)  % bearing linearization
            ] * Fi;

            % Measurement update
            K = S * transpose(Ht) * inv(Ht * S * transpose(Ht) + Qi);
            I = y(2 * (i - 1) + 1:2*i, t) - [rp; (atan2(dy,dx) - mu(3))];
            mu = mu + K * I;
            S = (eye(n + 2 * M) - K * Ht) * S;
        end
    end
 
    % Store results
    mu_S(:,t) = mu;

    % Plot results
    [newfeature] = plot_step(1, t, map, xr, mu, mu_S, S, y, newfeature);
end