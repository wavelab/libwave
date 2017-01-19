function kf = kf_measurement_update(kf, y)
    kf.mu = kf.mu_p + kf.K * (y - kf.C * kf.mu_p);
    kf.S = (eye(length(kf.A)) - kf.K * kf.C) * kf.S_p;
end
