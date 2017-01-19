addpath('plot');
addpath(genpath('../../../lib'));
close all

% parameters
nb_samples = 1000;
dxy = 0.1;
pos_start = [40/dxy 5/dxy pi];
pos_end = [50/dxy 10/dxy];

% prm
[map, map_min, map_max] = load_omap('IGVCmap.jpg');
milestones = prm_sample(nb_samples,...
	map,...
	pos_start,...
	pos_end,...
	map_min,... 
	map_max ...
);
[spath, sdist] = prm_search(map, milestones, 50, 1, 2);
plot_omap(1, map, pos_start, pos_end, 0.1);
plot_milestones(1, milestones);
for i = 1:length(spath) - 1
    plot([spath(i, 1), spath(i + 1, 1)], ...
        [spath(i, 2), spath(i + 1, 2)], ...
        'go-', ...
        'LineWidth', 3, ...
        'markersize', 5 ...
    );
end

% simulation parameters
t_end = 200;
dt = 0.01;  % 10 Hz
t = 0:dt:t_end;

% waypoints
waypoints = double(spath);

% model parameters
L = 0.3;
stddev_x = 0.02;
stddev_y = 0.02;
stddev_theta = deg2rad(1.0);
v_t = 3 / dxy;
 
% bicycle states
delta_max = deg2rad(25); % max steering angle
x_t = [
    pos_start(1);  % x
    pos_start(2);  % y
    pos_start(3);  % theta
];
x_store = zeros(length(x_t), length(t));
x_store(:, 1) = x_t;

% carrot states
carrot_t = [
    0;  % x
    0;  % y
];
carrot_store = zeros(length(carrot_t), length(t));
carrot_store(:, 1) = carrot_t;

% control parameters
k = 2.0;

% controller states
c_t = (0);  % heading error
c_store = zeros(length(c_t), length(t));
c_store(:, 1) = c_t;

% simulation
wp_index = 1;
delta_t = 0;

for i = 1:length(t)
    % update carrot
    [carrot_t, wp_index] = carrot_update( ...
        x_t, ...
        0.5 / dxy, ...
        waypoints, ...
        wp_index ...
    );
    carrot_store(:, i) = carrot_t;
    if wp_index == length(waypoints);
        break
    end
    
    % update steering
    delta_t = calculate_delta(x_t, carrot_t, delta_max);

    % update bicycle
    x_t = bicycle_update(x_t, v_t, L, delta_t, dt);
    x_store(:, i + 1) = x_t;
end

% plot animation
plot_waypoints(1, waypoints);
plot_trajectory(1, x_store, carrot_store, t);
%plot_animation(1, x_store, carrot_store, t);
