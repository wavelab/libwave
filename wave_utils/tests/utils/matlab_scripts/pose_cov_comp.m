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
    
    % must check what the output of these are and compare to the C++
    % ORIGINAL CODE
    % pR7 = [0.9990131207314632, 2.5, -3.062821518156257, 0.68555018611353924, 0.11964766099104401, 0.70743291328351732, -0.12346680826140885]
    % p17 = [-1, 2, -3, 6.1128606296199445e-16, 0.99987663248166059, -5.1606827940988524e-17, -0.015707317311820648];
    % p27 = [2, -0.5, 0, 0.12157223269087249, -0.69657748473415115, -0.12157223269087243, -0.69657748473415082]
    % p16 = [-1, 2, -3, -1.2246467991473532e-16, 0.031415926535897871, 3.1415926535897922]
    % p26 = [2, -0.5, 0, -2.7960174616949161, -1.5707963267948966, 0]
    
    % NEW CODE
    % pR7 = [0.9990131207314632, 2.5, -3.062821518156257, 0.68555018611353924, 0.11964766099104401, 0.70743291328351732, -0.12346680826140885]
    % p17 = [-1, 2, -3, 6.1128606296199445e-16, 0.99987663248166059, -5.1606827940988524e-17, -0.015707317311820648];
    % p27 = [2, -0.5, 0, -0.12157223269087249, 0.69657748473415115, 0.12157223269087243, 0.69657748473415082]
    % p16 = [-1, 2, -3, -1.2246467991473532e-16, 0.031415926535897871, 3.1415926535897922]
    % p26 = [2, -0.5, 0, 3.4871678454846706, -1.5707963267948966, 0]
    
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