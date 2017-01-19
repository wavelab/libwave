addpath('../../../lib/util');
addpath('../../../lib/sensor');

clear;
clc;

% Create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('fastwave.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 5;
    open(vidObj);
end

% Time
Tf = 20;
dt = 0.5;
T = 0:dt:Tf;

% Initial Robot State
x0 = [0; 0; 0];

% Motion Disturbance model
R = [0.001, 0, 0; 
     0, 0.001, 0; 
     0, 0, 0.0001];

% Control inputs
u = ones(2, length(T));
u(2, :) = 0.3 * u(2, :);

% Feature Map
M = 20;
map = 12 * rand(2, M);
map(1, :) = map(1, :) - 5.5; 
map(2, :) = map(2, :) - 2; 

% Measurement model
rmax = 10;
thmax = pi / 4;

% Feature initialization
newfeature = ones(M, 1);

% Measurement noise
Qi = [0.02 0; 
     0 0.02];

% Simulation Initializations
n = length(R(:, 1)); % Number of vehicle states
xr = zeros(n, length(T)); % Vehicle states 
xr(:, 1) = x0;
m = length(Qi(:, 1)); % Number of measurements per feature 
N = n + m * M; % Number of elements for belief

% Particles
D = 100;  % Number of particles
X = zeros(n,D);  % Vehicle states
X0 = X;  % Prior - known exactly
Xp = X;  % Interim Vehicle states
mu = zeros(2, M, D);  % Feature means
mup = mu;  % Interim Feature means
S = zeros(2, 2, M, D); % Feature covariances
Sp = S;  % Interim Feature covariances
mu_store = zeros(n, length(T));


% Initialization of Particle weights
w0 = 1 / D; 
w = w0 * ones(1, D);
    
% Main loop
for t = 2:length(T)
    % Update robot state
    e = gaussian_noise(R);
    xr(:, t) = [ ...
        xr(1, t - 1) + u(1, t) * cos(xr(3, t - 1)) * dt;
        xr(2, t - 1) + u(1, t) * sin(xr(3, t - 1)) * dt;
        xr(3, t - 1) + u(2, t) * dt
    ] + e;

    % Find and measure features
    y = [];
    meas_ind = find_features(map, xr(:, t), rmax, thmax);
    for i = meas_ind
        y = [y, feature_measure(map, i, xr(:, t), Qi)];
    end
    
    
    %% Fast wave Filter Estimation
    % Particle Filter Sampling
    for d = 1:D
        % Update full state
        em = gaussian_noise(R);
        Xp(:, d) = motion_model(X(:, d), u(:, t), dt) + em;

        % For each feature measured make an EKF
        clear yw;
        clear hmuw;
        clear Qw;
        for j = 1:length(meas_ind)
            i = meas_ind(j);
            
            if (newfeature(i) == 1)
                [mup_init, Sp_init, dx, dy, rp] = feature_initialize( ...
                    Xp(:, d), ...
                    y(:, j), ...
                    Qi ...
                );
                mup(:, i, d) = mup_init;
                Sp(:, :, i, d) = Sp_init;
                
            else
                [mup_update, Sp_update, dx, dy, rp] = feature_update( ...
                    Xp(:, d), ...
                    mu(:, i, d), ...
                    y(:, j), ...
                    Qi, ...
                    S(:, :, i, d) ...
                );
                mup(:, i, d) = mup_update;
                Sp(:, :, i, d) = Sp_update;
            end
            
            % Calculate predicted measurements and covariance for
            % weighting
            fi = 2 * (j - 1) + 1;
            fj = 2 * j;
            yw(fi:fj) = y(:, j);
            hmuw(fi:fj) = [
                rp; 
                mod(atan2(dy, dx) - Xp(3, d) + pi, 2 * pi) - pi
            ];
            Qw(fi:fj, fi:fj) = Sp(:, :, i, d);
        end
        
        % Calculate weight
        w(d) = w0;
        if (exist('Qw')) %#ok<EXIST>
            w(d) = max(0.000001, mvnpdf(yw, hmuw, Qw));
        end
    end
    
    % Eliminate initialization of previously measured features
    newfeature(meas_ind) = 0;
        
    % Resample and copy all data to new particle set
    [X, mu, S] = fastwave_resample(w, Xp, mup, Sp, n, M, D);
    muParticle = mean(transpose(X));
    mu_store(:, t) = muParticle;

    % Plot results
    plot_results(1, t, xr, mu, map, y, meas_ind, newfeature, X, M, D);
    if (makemovie) 
        writeVideo(vidObj, getframe(gca)); 
    end
end
if (makemovie) 
    close(vidObj); 
end
