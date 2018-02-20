clear;


% Test cases
% v = [1 0 0 0];
v = [0.182574185835055 0.365148371670111 0.547722557505166 0.730296743340221];

% Numerical Jacobian
eps = 1e-10;
e = sqrt(eps);
p = eye(4)*e;
J_numerical = zeros(3,4);

fxn = @(qr, qx, qy, qz) [atan2(2 * (qr*qz + qx*qy), 1 - 2*(qy^2 + qz^2));
                         asin(2 * (qr*qy - qx*qz));
                         atan2(2* (qr*qx + qy*qz), 1 - 2*(qx^2 + qy^2))];


 xc = [0, 0, 0];   
 xp = [0, 0, 0];   
 
xc = fxn(v(1), v(2), v(3), v(4));

for i = 1:4
    vp = v + p(i,:); % add perturbation
    % get perturbed matrix

    xp = fxn(vp(1), vp(2), vp(3), vp(4));

    J_numerical(:,i) = (xp'-xc')./e;
end

if (abs(J_numerical - jacobian_Quat_Norm_to_Rpy_wrt_q(v)) < 1e-4*ones(3,4))
    disp 'Numerical solution equal analytical';
else
    disp 'Numerical solution NOT equal analytical';
end