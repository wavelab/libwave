function [ekf] = ekf_feature_measurement_update(ekf, h_func, H_func, mf, y, data)
	h = feval(h_func, mf, y, ekf.mu_p, data);
    H = feval(H_func, mf, y, ekf.mu_p, data);

    % kalman gain
	K = ekf.S_p * transpose(H) * inv(H * ekf.S_p * transpose(H) + ekf.Q); %#ok<MINV>
    
    % innovation or measurement residual
    if isfield(ekf, 'I') == 1
        I = h;
    else
        I = (y - h);
    end
    
    % update state estimate
    ekf.mu = ekf.mu_p + K * I;
   
    % update covariance estimate
    ekf.S = (eye(length(ekf.mu)) - K * H) * ekf.S_p;
end
