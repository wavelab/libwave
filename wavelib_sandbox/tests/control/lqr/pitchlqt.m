%% Linear Quadratic Tracking - Pitch Control of Aircraft
addpath('plot');
addpath('../../../lib/control');


%% Continuous time model
% State x = [angle of attack, pitch rate, pitch]
A = [-0.313, 56.7, 0; -0.0139, -0.426, 0; 0, 56.7, 0];
B = [0.232; 0.0203; 0];
C = [0, 0, 1];
D = [0];
% A = [0 1 0; 0 0 1; -0.1 0 1.4];
% B = [0; 0; .24];
% C = eye(3);
% D = zeros(3,1);
% systf = tf(conv([1 2.5],[1 0.7]),conv([1 5 40],[1 0.03 0.06]));
% [A,B,C,D] = tf2ss(systf);
sys = ss(A, B, C, D);

% Discretize
dt = 0.1;
sysd = c2d(sys, dt, 'zoh');
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

%% Desired Trajectory Simulation
t0 = 0;
tf = 20;
T = t0:dt:tf;


% generate desired states
x0 = [1 0 0];
xd0 = [0 0 0];
xd = zeros(3, length(T));
xd(:,1) = xd0';
ud = sin(T(1:end-1)); % desired input
% ud = zeros(1, length(T));  % desired input - set to zero
for t = 1:length(T)-1
    xd(:, t + 1) = Ad * xd(:, t) + Bd * ud(:, t);
end


% LQR costs
Q = 0.01 * eye(3);
R = 0.01 * eye(1);

% Costate setup
P = zeros(3, 3, length(T));
P_S(:, :, length(T)) = Q;
Pn = P_S(:, :, length(T));

% Solve for costate
for t=length(T)-1:-1:1
    P = Q + Ad' * Pn * Ad - Ad' * Pn * Bd * inv(Bd'*Pn*Bd+R) * Bd' * Pn * Ad;
    P_S(:, :, t) = P;
    Pn = P;
end


% Solve for control gain and simulate
x = zeros(3, length(T));
x(:, 1) = x0';
u = zeros(1, length(T) - 1);

% Error regulation
[dx, du, Jx, Ju, P, K] = lqr_sim(Ad, Bd, Q, R, t0, tf, dt, x(:,1) - xd(:,1), 0);
% dx : delta state
% du : delta input
% Jx : state cost
% Ju : input cost
% P : costate
% K : gain


for t = 1:length(T) - 1
    u(:,t) = du(:, t) + ud(:, t);
    x(:, t + 1) = dx(:, t + 1) + xd(:, t + 1);
end

figure(1);
hold on;
clf;

subplot(4,1,1);
hold on;
plot(T, xd(1, :), 'b--');
plot(T, x(1, :), 'b-');
title('Angle of Attack');

subplot(4,1,2);
hold on;
plot(T, xd(3, :), 'b--');
plot(T, x(3, :), 'b-');
title('Pitch Angle');

subplot(4,1,3);
hold on;
plot(T, xd(2, :), 'b--');
plot(T, x(2, :), 'b-');
title('Pitch Rate');

subplot(4,1,4);
hold on;
plot(T(1:end-1), ud(:), 'b--');
plot(T(1:end-1), u(:), 'B-');
title('Deflection Angle');