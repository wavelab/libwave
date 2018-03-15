% pose input is [x y z roll pitch yaw] as is the covariance matrix
function [predicted_cov, actual_cov, error_plot] = ...
    compare_predicted_vs_actual_covariance(p1, p1_cov, ...
                                           p2, p2_cov, ...
                                           num_samples)
    p1_pos = [p1(1), p1(2), p1(3)];
    p1_eul = p1(4:end);
    p1_rot = eul2rotm([p1(6), p1(5), p1(4)]);

    p2_pos = [p2(1), p2(2), p2(3)];
    p2_eul = p1(4:end);
    p2_rot = eul2rotm([p2(6), p2(5), p2(4)]);

    p1_samples = mvnrnd(p1, p1_cov, num_samples);
    p2_samples = mvnrnd(p2, p1_cov, num_samples);
    rs = zeros(num_samples, 6);
    r_cov_sum = zeros(size(p1_cov));

    [r_pos, r_rot, r_cov_predicted] = pose_cov_comp(p1_pos', p1_rot, p1_cov, p2_pos', p2_rot, p2_cov);
    
    point = 1;
    num_points = 100;
    error_plot = zeros(num_points, 2);

    for i = 1:num_samples
        p1_pos = p1_samples(i, 1:3);
        p1_eul = p1_samples(i, 4:end);
        p1_rot = eul2rotm([p1_eul(3), p1_eul(2), p1_eul(1)]);

        p2_pos = p2_samples(i, 1:3);
        p2_eul = p2_samples(i, 4:end);
        p2_rot = eul2rotm([p2_eul(3), p2_eul(2), p2_eul(1)]);

        [r_pos, r_rot, r_cov] = pose_cov_comp(p1_pos', p1_rot, p1_cov, p2_pos', p2_rot, p2_cov);
        r_eul = quatToRpy(rotm2quat(r_rot));
        rs(i, :) = [r_pos', r_eul(3), r_eul(2), r_eul(1)];
        r_cov_sum = r_cov_sum + r_cov;
        
        if i / num_samples >= point/num_points
            predicted_cov = r_cov_predicted;
            computed_rs = rs;
            computed_rs(~any(computed_rs,2),:) = [];
            actual_cov = cov(computed_rs);
            error = norm(predicted_cov - actual_cov)/norm(actual_cov);
            error_plot(point,:) = [i, error]; 
            point = point + 1;
        end
    end
    
    predicted_cov = r_cov_predicted;
    actual_cov = cov(rs);
    error_plot(~any(error_plot,2), :) = [];
end