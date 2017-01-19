%% Trajectory rollout example
clear; 
clc;
addpath('../../../lib/planning/prm');


%% Problem parameters
tic;

% Set up the map
xMax = [32 30];  % State bounds
xMin = [0 0];
xR = xMax-xMin;

% Set up the goals
x0 = [25.5, 28.5, -3.0];
xF = [6, 1];

% Set up the obstacles
%rand('state', 1);
nO = 25;  % number of obstacles
nE = 4;  % number of edges per obstacle (not changeable).
minLen.a = 1;  % Obstacle size bounds
maxLen.a = 6;
minLen.b = 2;
maxLen.b = 8;

obstBuffer = 0; % Buffer space around obstacles
maxCount = 1000; % Iterations to search for obstacle locations

% Find obstacles that fit
[aObsts, bObsts, obsPtsStore] = polygonal_world( ...
    xMin, ...
    xMax, ...
    minLen, ...
    maxLen, ...
    nO, ...
    x0, ...
    xF, ...
    obstBuffer, ...
    maxCount ...
);

% Define single freespace nonconvex polygon by its vertices, each hole
% separated by NaNs
env = [
    xMin(1) xMin(2);
    xMin(1) xMax(2);
    xMax(1) xMax(2);
    xMax(1) xMin(2); 
    xMin(1) xMin(2)
];
obsEdges = [];
for i = 1:nO
    env = [
        env; 
        NaN NaN; 
        obsPtsStore(:, 2*(i-1)+1:2*i);
        obsPtsStore(1, 2*(i-1)+1:2*i)
    ];
    obsEdges = [ ...
        obsEdges;  ...
        obsPtsStore(1:nE,2*(i-1)+1:2*i), ...
        obsPtsStore([2:nE 1],2*(i-1)+1:2*i) ...
    ];
end

% Plot obstacles
figure(1); 
clf; 
hold on;
plotEnvironment(obsPtsStore, xMin, xMax, x0, xF);
drawnow();
figure(1); hold on;
disp('Time to create environment');
toc;

% create AVI object
% vidObj = VideoWriter('traj_rollout4.avi');
% vidObj.Quality = 100;
% vidObj.FrameRate = 5;
% open(vidObj);

% Vehicle
dt = 0.01;
uMin = [2 -2]; % bounds on inputs, [velocity, rotation rate]
uMax = [2 2]; % bounds on inputs, [velocity, rotation rate]
uR = uMax - uMin; % range of inputs
sMin = 20; % steps to move
sMax = 100; % steps to compute for rollout
n_traj = 15; % number of trajectories to roll out

% Trajectory rollout, created until solution found
T = 120;
trajectory_rollout(T, dt, env, x0, xF, uR, uMin, uMax, sMin, sMax, obsEdges, nO, n_traj, 0);
% close(vidObj);