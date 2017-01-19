addpath('../../../lib/util');
addpath('../../../lib/sensor');

%% Scan Registration example
clear;
clc;

% Time
Tf = 50;
dt = 0.5;
T = 0:dt:Tf;

% True map
M = 1000;
N = 1000;
dx = 0.1;
I = imread('funnymap.jpg');
I = imresize(I, [M N]);
map = im2bw(I, 0.7); % Convert to 0-1 image
map = 1-flipud(map'); % Convert to 0 free, 1 occupied and flip.

% Initial Robot State
x0 = [25 20 pi/2]';

% Motion Disturbance model
R = [0.001 0 0; 
     0 0.001 0; 
     0 0 0.0001];

% Control inputs
u = zeros(2, length(T));
u(1,:) = 5;
u(2,:) = -0.05;

% Measurement model
rmax = 40;
meas_phi = [-pi/1.1:0.05:pi/1.1];
Q = 0.0002;
d = gaussian_noise(Q);
x0s = [x0(1)/dx; x0(2)/dx; x0(3)];
meas_r = getranges(map,x0s,meas_phi,rmax/dx);

% Make first measurement point cloud
kk = 1;
for jj = 1:length(meas_phi)
    if(meas_r(jj) < (rmax / dx))
        pp(kk, :) = [
            dx * meas_r(jj) * cos(meas_phi(jj)), ...
            dx * meas_r(jj) * sin(meas_phi(jj)) ...
        ];
        kk = kk + 1;
    end
end

% Simulation Initializations
n = length(R(:,1)); % Number of vehicle states
xr = zeros(n,length(T)); % Vehicle states 
xr(:,1) = x0;
xICP = x0;

% Belief map
m = 0.5 * ones(M, N);
L0 = log(m ./ (1 - m));
L = L0;

% Motion planning look ahead
dxy1 = 5;
dxy2 = 15;


%% Main loop
[meas_r_M, meas_r_N] = size(meas_r);
meas_r = zeros(meas_r_N, length(T));

for t = 2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = gaussian_noise(R);
    % Update robot state
    xr(:,t) = [xr(1,t-1)+u(1,t)*cos(xr(3,t-1))*dt;
              xr(2,t-1)+u(1,t)*sin(xr(3,t-1))*dt;
              xr(3,t-1)+u(2,t)*dt] + e;

    % Jump through obstacles if not avoided
    xc = max(1, min(N, round(xr(1,t)/dx)));
    yc = max(1, min(M, round(xr(2,t)/dx)));
    if (map(xc, yc)) 
        xICP(:,t) = xr(:,t);
        continue; 
    end
    
    % Define measurements
    d = gaussian_noise(Q);
    xrs = [xr(1, t)/dx; xr(2, t)/dx; xr(3, t)];
    meas_r(:, t) = getranges(map, xrs, meas_phi, rmax/dx);

    % Make current measurement point cloud
    kk = 1; 
    ppcur = [];
    for jj = 1:length(meas_phi)
        if (meas_r(jj, t) < (rmax / dx)) % check range is not beyond rmax
            ppcur(kk, :) = [
                dx * meas_r(jj, t) * cos(meas_phi(jj)), ...
                dx * meas_r(jj, t) * sin(meas_phi(jj)) ...
            ]; 
            kk = kk + 1;
        end
    end
    
    
    %% ICP
    % Store previous and current point cloud
    qq = pp;
    pp = ppcur;
   
    % Predicted transformation
    Rp = [
        cos(u(2, t)), -sin(u(2, t)); 
        sin(u(2, t)), cos(u(2, t))
    ];
    Tp = [u(1, t) * dt; 0];

    
    % Perform ICP scan registration
    [Ricp, Ticp] = icp2(qq', pp', Rp, Tp, 5);
    
    % Store robot pose estimate
    x_cur = xICP(:, t - 1);
    Rth = [cos(x_cur(3)) -sin(x_cur(3)); sin(x_cur(3)) cos(x_cur(3))];
    xICP(1:2,t) = x_cur(1:2) + Rth * Ticp;
    xICP(3,t) = x_cur(3) + asin(Ricp(2, 1));
    
    % Transform pointcloud for display
    pp2 = [];
    for kk = 1:length(pp)
        pp2(kk, :) = (Ricp * pp(kk, :)' + Ticp)'; 
    end
    
    % record error between true state and estimated pose from ICP
    diffT(:, t) = xr(1:2, t) - xr(1:2, t - 1);
    diffICP(:, t) = Ticp;
    errICP(t) = norm(diffT(:, t) - diffICP(:, t));
    errICP(t);
    
    % map update
    measL = zeros(M,N);
    for i = 1:length(meas_phi)
        % Get inverse measurement model (with scan registration estimate or
        % motion prediction)
        invmod = inversescannerbres( ...
            M, ...
            N, ...
            xICP(1, t) / dx, ...
            xICP(2, t) / dx, ...
            meas_phi(i) + xICP(3, t), ...
            meas_r(i, t), ....
            rmax / dx ...
        );
    
        for j = 1:length(invmod(:,1));
            ix = invmod(j,1);
            iy = invmod(j,2);
            il = invmod(j,3);
            
            % Calculate updated log odds
            L(ix, iy) = L(ix, iy) + log(il ./ (1 - il)) - L0(ix, iy);
            measL(ix, iy) = measL(ix, iy) + log(il ./ (1 - il)) - L0(ix, iy);
        end
    end
    
    % Calculate probabilities
    m = exp(L) ./ (1 + exp(L));
    invmod_T = exp(measL) ./ (1 + exp(measL));

    % Control modification for collisions
    front = find(meas_phi > 0,1);
    if (meas_r(front, t) < 25 / dx)
        u(2, t + 1) = -0.75;
    end
    
    % plot results
    plot_results( ...
        t, ...
        dx, ...
        map, ...
        xr, ...
        xICP, ...
        pp, ...
        pp2, ...
        qq, ...
        invmod_T, ...
        meas_r, ...
        meas_phi, ...
        m, ...
        M, ...
        N ...
    );
end