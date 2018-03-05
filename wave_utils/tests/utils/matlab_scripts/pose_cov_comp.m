function [r_pos, r_rot, r_cov] = pose_cov_comp(p1_pos, p1_rot, p1_cov, p2_pos, p2_rot, p2_cov)
    r_pos = zeros(3, 1);
    r_rot = zeros(3, 3);
    r_cov = zeros(6, 6);
    
    T_p1 = [p1_rot, p1_pos;
            0, 0, 0, 1];
    T_p2 = [p2_rot, p2_pos;
            0, 0, 0, 1];
    T_r = T_p1 * T_p2;
    
    r_rot = T_r(1:3, 1:3);
    r_pos = T_r(1:3, 4);
    
    cov_p1 = p1_cov;
    cov_p2 = p2_cov;
  
    p1_quat = rotm2quat(p1_rot);
    p2_quat = rotm2quat(p2_rot);
    r_quat = rotm2quat(r_rot);
    
    p1_eul = rotm2eul(p1_rot);
    p2_eul = rotm2eul(p2_rot);
    
    pR7 = [r_pos', r_quat];
    p17 = [p1_pos', p1_quat];
    p27 = [p2_pos', p2_quat];
    
    p16 = [p1_pos' quatToRpy(p1_quat)'];
    p26 = [p2_pos' quatToRpy(p2_quat)'];
    
    jacobian_p7_to_p6 = jacobian_p7_to_p6_wrt_p(pR7);
    jacobian_p7_p7_composition = jacobian_p7_p7_Composition_wrt_p1(p17, p27);
    jacobian_p6_to_p7 = jacobian_p6_to_p7_wrt_p(p16);
    dfpc_dp = jacobian_p7_to_p6 * jacobian_p7_p7_composition * jacobian_p6_to_p7;

    % Equation (5.4)
    jacobian_p7_p7_composition = jacobian_p7_p7_Composition_wrt_p2(p17, p27);
    jacobian_p6_to_p7 = jacobian_p6_to_p7_wrt_p(p26);
    dfpc_dq = jacobian_p7_to_p6 * jacobian_p7_p7_composition * jacobian_p6_to_p7;

    % Equation (5.2)
    % putting everything together
    final_cov = dfpc_dp * cov_p1 * dfpc_dp' + ...
                dfpc_dq * cov_p2 * dfpc_dq';

    r_cov = final_cov;
end