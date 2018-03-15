clear;

% Test cases
% v = [1 0 0 0];
v = [0 0 0 0.182574185835055 0.365148371670111 0.547722557505166 0.730296743340221]';

% Numerical jacobian
fxn1 = @(k) [k(1); k(2); k(3);
             atan2(2 * (k(4)*k(7) + k(5)*k(6)), 1 - 2*(k(6)^2 + k(7)^2));
             asin(2 * (k(4)*k(6) - k(5)*k(7)));
             atan2(2* (k(4)*k(5) + k(6)*k(7)), 1 - 2*(k(5)^2 + k(6)^2))];
                               
fxn2 = @(k) [k(1); k(2); k(3); quatnormalize([k(4) k(5) k(6) k(7)])'];

fxn = @(k) fxn1(fxn2(k));

eps = 1e-10;
e = sqrt(eps);
p = eye(7)*e;
J_numerical = zeros(6,7);
                     
xc = fxn(v);

for i = 1:7
    vp = v + p(:,i); % add perturbation
    % get perturbed matrix
    xp = fxn(vp);
    J_numerical(:,i) = (xp-xc)./e;
end

if (abs(J_numerical - jacobian_p7_to_p6_wrt_p(v)) < 1e-4*ones(6,7))
    disp 'Numerical solution equal analytical';
else
    disp 'Numerical solution NOT equal analytical';
end