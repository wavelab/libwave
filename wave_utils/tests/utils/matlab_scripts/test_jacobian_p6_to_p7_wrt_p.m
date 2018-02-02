clear;

% Test Cases
v = [4 3 -1 0.942477796076938 -15.707963267948966  -0.031415926535898]';
% v = [1 2 3 0.182574185835055 0.365148371670111 0.547722557505166 0.730296743340221]';

% Generate numerical solutions
F = @(p) [p(1) p(2) p(3) angle2quat(p(4), p(5), p(6))]';
eps = 1e-10;
e = sqrt(eps);
p = eye(6)*e;
J_numerical = zeros(7,6);

xc = F(v);

for i = 1:6
    vp = v + p(:,i); % add perturbation
    xp = F(vp); % get perturbed matrix
    J_numerical(:,i) = (xp-xc)./e;
end

if (abs(J_numerical - jacobian_p6_to_p7_wrt_p(v)) < 1e-4*ones(7,6))
    disp 'Numerical solution equal analytical';
else
    disp 'Numerical solution NOT equal analytical';
end