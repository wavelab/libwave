function g_t = g(u, x, dt, data)
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
		cos(x(3)), -sin(x(3)), 0;
		sin(x(3)), cos(x(3)), 0;
		0, 0, 1;
	];

	gdot_t = inv(rot) * inv(J_1) * J_2 * u;
    g_t = x + gdot_t * dt;
end