% RANSAC correspondence outlier rejection for measurement update
clear; 
clc; 
rng('default');

addpath('plot');
addpath('../../../lib/sensor');

% Features
N = 300;
map = 100 * rand(N,2);

% Robot parameters
x0 = [15, 15, (45 * pi / 180)];
rmax = 50;
thmax = 60 * pi / 180;
Q = 0.0001;
mu0 = [10, 20, (90 * pi / 180)];  % Initial estimate of pose

% Visible features measurement vector
[meas_list, meas_raw, meas_ind, meas_count] = measure_features( ...
    N, ...
    map, ...
    x0, ...
    rmax, ...
    thmax, ...
    Q ...
);

% Incorrect correspondence insertion 
[outlier_ind, outlier_count, meas_ind, meas_ind_old] = generate_incorrect_correspondence( ...
    meas_ind, ...
    meas_count, ...
    0.2 ...
);


% RANSAC
nlls_max_iterations = 50;
ransac_max_iterations = 100;
sample = 5;
inlier_threshold = 0.03;
max_set_length = 0;
max_set = [];
r_size = zeros(1, ransac_max_iterations);
r_error = zeros(1, ransac_max_iterations);

for k = 1:ransac_max_iterations
    % Initialize estimate
    mu = transpose(mu0);
    mu_s = zeros(3, nlls_max_iterations + 1);
    mu_s(:, 1) = mu;

    % Pick samples to use for solution
    sample_ind = ceil(meas_count * rand(sample, 1));
    
    % Solve NLLS to find robot position
    meas_sample = meas_raw(sample_ind);
    for m = 1:nlls_max_iterations
        H = zeros(length(sample), 3);
        rp = zeros(length(sample));
        thetap = zeros(length(sample));
        
        for n = 1:sample
            cur_pt = map(meas_ind(sample_ind(n)), :);
            
            % measurement model
            dx = cur_pt(1) - mu(1);
            dy = cur_pt(2) - mu(2);
            rp(n) = sqrt(dx^2 + dy^2);
            thetap(n) = atan2(dy, dx) - mu(3);
            
            % linearize measurement model
            H(n, :) = [ ...
                (dy / rp(n)^2), ...
                (-dx / rp(n)^2), ...
                -1 ...
            ];
        end
        
        % solve non-linear least squares
        mu = mu + pinv(H) * (transpose(meas_sample) - transpose(thetap));
        mu_s(:, m + 1) = mu;
    end

    % Find inlier set
    inlier_set = [];
    for p = 1:meas_count
        if (find(sample_ind == p))
            inlier_set = [inlier_set; p];
            
        else
            cur_pt = map(meas_ind(p), :);
                        
            % measurement model on belief vs measured
            dx = cur_pt(1) - mu(1);
            dy = cur_pt(2) - mu(2);
            thetac =  atan2(dy, dx) - mu(3);
            error = abs(meas_raw(p) - thetac);
            
            % check if error is within inlier threshold
            if (error < inlier_threshold)
                inlier_set = [inlier_set; p];
            end            
        end
    end

    % Save biggest inlier set
    if (length(inlier_set) > max_set_length)
        max_set_length = length(inlier_set);
        max_set = inlier_set;
    end
    r_size(k) = length(inlier_set);
    r_error(k, :) = min(100, norm(mu - transpose(x0)));
end

% Resolve NLLS with biggest inlier set (iterate on measurement update)
meas_set = meas_raw(max_set);
for m = 1:nlls_max_iterations
    H = zeros(length(max_set_length), 3);
    
    for n = 1:max_set_length
        cur_pt = map(meas_ind(max_set(n)),:);
        
        % measurement model
        dx = cur_pt(1) - mu(1);
        dy = cur_pt(2) - mu(2);
        rp(n) = sqrt(dx^2 + dy^2);
        thetap(n) = atan2(dy, dx) - mu(3);
        
        % linearize measurment model
        H(n, :) = [(dy / rp(n)^2), (-dx / rp(n)^2), -1];
    end
    
    mu = mu + pinv(H) * (transpose(meas_set) - transpose(thetap));
    mu_s(:, m + 1) = mu;
end

% Plot NLLS result for current seed set
figure(1); 
clf; 
hold on;
plot_visible_features(1, map, meas_ind, x0);
plot_outliers(2, map, meas_ind, meas_ind_old, outlier_count);
plot_hypothesis(3, map, x0, mu, sample_ind, meas_ind, meas_sample);
plot_nlls_result(4, map, x0, meas_ind, mu_s, max_set, max_set_length);