eps = 1e-10;

% Test cases
% v = [1 0 0 0];
% v = [0.182574185835055 0.365148371670111 0.547722557505166 0.730296743340221];
v = [-1, 3, -10, -5];

% Numerical Jacobian
e = sqrt(eps);
p = eye(4)*e;
J_numerical = zeros(4,4);

xc = quatnormalize(v);

for i = 1:4
    vp = v + p(i,:); % add perturbation
    % get perturbed matrix
    xp = quatnormalize(vp);
    J_numerical(:,i) = (xp'-xc')./e;
end

if (abs(J_numerical - jacobian_Quat_Norm_wrt_q(v)) < 1e-4*ones(4,4))
    disp 'Numerical solution equal analytical';
else
    disp 'Numerical solution NOT equal analytical';
end