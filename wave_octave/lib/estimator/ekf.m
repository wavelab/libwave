function x = EKF(dt, mu, R, Q, S)
	x = struct(
		"dt", dt,

		"mu", mu,
		"S", S,
		"R", R,
		"Q", Q,

		"mu_p", 0.0,
		"S_p", 0.0,
		"K", 0.0
	);
end

function [ekf] = ekf_prediction_update(ekf, g_func, G_func, u)
	g = feval(g_func, u, ekf.mu, ekf.dt);
	G = feval(G_func, u, ekf.mu, ekf.dt);

	ekf.mu_p = g;
	ekf.S_p = G * ekf.S * G' + ekf.R;
end

function [ekf] = ekf_measurement_update(ekf, h_func, H_func, y)
	h = feval(h_func, ekf.mu_p);
	H = feval(H_func, ekf.mu_p);

	K = ekf.S_p * transpose(H) * inv(H * ekf.S_p * transpose(H) + ekf.Q);
	ekf.mu = ekf.mu_p + K * (y - h);
    ekf.S = (eye(length(ekf.mu)) - K * H) * ekf.S_p;
end
