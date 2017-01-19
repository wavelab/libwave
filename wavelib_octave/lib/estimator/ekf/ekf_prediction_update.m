function [ekf] = ekf_prediction_update(ekf, g_func, G_func, u, data)
	g = feval(g_func, u, ekf.mu, ekf.dt, data);
	G = feval(G_func, u, ekf.mu, ekf.dt, data);

	ekf.mu_p = g;
	ekf.S_p = G * ekf.S * transpose(G) + ekf.R;
end
