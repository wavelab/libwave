function J =   jacobian_p7_to_p6_wrt_p(p)

J = eye(6,7);


J(4:6, 4:7) = jacobian_Quat_Norm_to_Rpy_wrt_q(p(4:7));

J(4:6, 4:7) = J(4:6, 4:7) * jacobian_Quat_Norm_wrt_q(p(4:7));
