
clc
close all
% simulation parameters
t_end = 20;
dt = 0.1;  % 10 Hz
t = 0:dt:t_end;

% waypoints
rect_x = 5;
rect_y = 10;
rect_w = 20;
rect_h = 5;

wp_1 = [rect_x, rect_y];
wp_2 = [rect_x + rect_w, rect_y];
wp_3 = [rect_x + rect_w, rect_y - rect_h];
wp_4 = [rect_x, rect_y - rect_h];

% wp_2 = [50.0, 100.0];
% waypoints = [wp_1; wp_2; wp_1];
waypoints = [wp_1; wp_2; wp_3; wp_4; wp_1];

% model parameters
L = 0.3;
stddev_x = 0.02;
stddev_y = 0.02;
stddev_theta = deg2rad(1.0);
v_t = 3;

% bicycle states
delta_max = deg2rad(25); % max steering angle
x_t = [
    22;  % x
    10;  % y
    deg2rad(0);  % theta
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
c_t = [0;];  % heading error
c_store = zeros(length(c_t), length(t));
c_store(:, 1) = c_t;




% simulation
wp_index = 1;
delta_t = 0;

for i = 1:length(t)
    % update carrot
    [carrot_t, wp_index] = carrot_update(x_t, carrot_t, 2, waypoints, wp_index);
    carrot_store(:, i) = carrot_t;

    % update steering
    delta_t = calculate_delta(x_t, carrot_t, delta_max);

    % update bicycle
    x_t = bicycle_update(x_t, v_t, L, delta_t, dt);
    x_store(:, i + 1) = x_t;
end

% plot animation
% plot_waypoints(1, waypoints);
% plot_trajectory(1, x_store, carrot_store, t);
plot_animation(1, x_store, carrot_store, t);
% plot_controller(1, c_store, t);

