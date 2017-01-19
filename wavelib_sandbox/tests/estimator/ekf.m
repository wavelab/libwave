source('lib/util.m');
source('lib/estimator/ekf.m');

function G_t = G(u, mu, dt)
	% length from wheel to center of chassis
	l = 0.3;

	% radius of wheel
	r = 0.25;

	% wheel constraints
	J_1 = [
		0, 1, l;
		-cos(pi / 6), -sin(pi / 6), l;
		cos(pi / 6), -sin(pi / 6), l;
	];

	% wheel radius
	J_2 = [
		r, 0.0, 0.0;
		0.0, r, 0.0;
		0.0, 0.0, r;
	];

	% constant terms
	C = inv(J_1) * J_2 * u * dt;

	% rotation matrix
	drot = [
		-sin(mu(3)), cos(mu(3)), 0;
		-cos(mu(3)), -sin(mu(3)), 0;
		0, 0, 0;
	];
	drot_C = drot * C;

	% fill in the jacobian matrix G
	G_t = eye(3);
	G_t(1:2, 3) = drot_C(1:2);
end

function g_t = g(u, mu, dt)
	% length from wheel to center of chassis
	l = 0.3;

	% radius of wheel
	r = 0.25;

	% wheel constraints
	J_1 = [
		0, 1, l;
		-cos(pi / 6), -sin(pi / 6), l;
		cos(pi / 6), -sin(pi / 6), l;
	];

	% wheel radius
	J_2 = [
		r, 0.0, 0.0;
		0.0, r, 0.0;
		0.0, 0.0, r;
	];

	% rotation matrix
	rot = [
		cos(mu(3)), -sin(mu(3)), 0;
		sin(mu(3)), cos(mu(3)), 0;
		0, 0, 1;
	];

	gdot_t = inv(rot) * inv(J_1) * J_2 * u;
    g_t = mu + gdot_t * dt;
end

function h = h(mu_p)
	H = eye(3);
	h = H * mu_p;
end

function H = H(mu_p)
	H = eye(3);
end

function simulation_loop(dt, nb_states, ekf)
    % simulation parameters
    t_end = 10;
    T = 0:dt:t_end;
    x_0 = [0; 0; 0];
    x = zeros(nb_states, length(T));
    x(:, 1) = x_0;
    y = zeros(length(ekf.Q), length(T));
	u = [0.0; -10.0; 10.0];

    % store
    mu = zeros(length(x_0));
    noise = zeros(length(ekf.R), length(x_0));

	% simulation
    for t = 2:length(T)
        % update state
        e = gaussian_noise(ekf.R);
        x(:, t) =  g(u, x(:, t - 1), dt) + e;
        noise(:, t) = e;

        % take measurement
        d = gaussian_noise(ekf.Q);
        y(:, t) = x(:, t) + d;

		% EKF
		[ekf] = ekf_prediction_update(ekf, @g, @G, u);
		[ekf] = ekf_measurement_update(ekf, @h, @H, y(:, t));

		% store ekf results
		mu(:, t) = ekf.mu;
    end

    plot_trajectory_2d(T, x, y, mu);
end


% simulation parameters
dt = 0.1;
mu = [0; 0; 0];
R = [
	0.05^2, 0, 0;
	0, 0.05^2, 0;
	0, 0, deg2rad(0.5)^2;
];
Q = [
	0.5^2, 0, 0;
	0, 0.5^2, 0;
	0, 0, deg2rad(10)^2;
];
S = eye(3);


% ekf
ekf = EKF(dt, mu, R, Q, S);
simulation_loop(dt, 3, ekf);
