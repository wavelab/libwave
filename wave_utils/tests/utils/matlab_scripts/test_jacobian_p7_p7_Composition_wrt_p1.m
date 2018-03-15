clear;

% Test case
p1 = [4 3 -1 0.475528258147577  -0.154508497187474   0.267616567329817  -0.823639103546332]';
p2 = [1 2 3 0.182574185835055 0.365148371670111 0.547722557505166 0.730296743340221]';

% Test Numerical solution
fxn = @(p, q) [p(1) + q(1) + 2 * (-((p(6) * p(6) + p(7) * p(7)) * q(1)) + ((p(5) * p(6) - p(4) * p(7)) * q(2)) +((p(4) * p(6) + p(5) * p(7)) * q(3)))
               p(2) + q(2) + 2 * (((p(4) * p(7) + p(5) * p(6)) * q(1)) - ((p(5) * p(5) + p(7) * p(7)) * q(2)) + ((p(6) * p(7) - p(4) * p(5)) * q(3)))
               p(3) + q(3) + 2 * (((p(5) * p(7) - p(4) * p(6)) * q(1)) + ((p(4) * p(5) + p(6) * p(7)) * q(2)) - ((p(5) * p(5) + p(6) * p(6)) * q(3)))
               (p(4) * q(4)) - (p(5) * q(5)) - (p(6) * q(6)) - (p(7) * q(7))
               (p(4) * q(5)) + (q(4) * p(5)) + (p(6) * q(7)) - (q(6) * p(7))
               (p(4) * q(6)) + (q(4) * p(6)) + (p(7) * q(5)) - (q(7) * p(5))
               (p(4) * q(7)) + (q(4) * p(7)) + (p(5) * q(6)) - (q(5) * p(6))];
n = @(p) [p(1) p(2) p(3) quatnormalize(p(4:7,:)')]';           
f = @(p, q) fxn(n(p), n(q));

eps = 1e-10;
e = sqrt(eps);
p = eye(7)*e;
J_numerical = zeros(7,7);

xc = f(p1, p2);

for i = 1:7
    vp = p1 + p(:,i); % add perturbation
    % get perturbed matrix
    xp = f(vp, p2);
    J_numerical(:,i) = (xp-xc)./e;
end

if (abs(J_numerical - jacobian_p7_p7_Composition_wrt_p1(p1, p2)) < 1e-4*ones(7,7))
    disp 'Numerical solution equal analytical';
else
    disp 'Numerical solution NOT equal analytical';
end