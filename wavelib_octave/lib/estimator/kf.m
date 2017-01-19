function x = KF(mu, S, A, B, C, R, Q)
	x = struct(
		"mu", mu,
		"S", S,

		"A", A,
		"B", B,
		"C", C,

		"R", R,
		"Q", Q,

		"mu_p", 0.0,
		"S_p", 0.0,
		"K", 0.0
	);
end

function kf = kf_prediction_update(kf, u)
    kf.mu_p = kf.A * kf.mu + kf.B * u;
    kf.S_p = kf.A * kf.S * transpose(kf.A) + kf.R;
    kf.K = kf.S_p * kf.C * inv(kf.C * kf.S_p * transpose(kf.C) + kf.Q);
end

function kf = kf_measurement_update(kf, y)
    kf.mu = kf.mu_p + kf.K * (y - kf.C * kf.mu_p);
    kf.S = (eye(length(kf.A)) - kf.K * kf.C) * kf.S_p;
end

function kf = kf_estimate(kf, u, y)
    kf = kf_prediction_update(kf, u);
    kf = kf_measurement_update(kf, y);
end
