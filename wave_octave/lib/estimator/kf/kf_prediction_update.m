function kf = kf_prediction_update(kf, u)
    kf.mu_p = kf.A * kf.mu + kf.B * u;
    kf.S_p = kf.A * kf.S * transpose(kf.A) + kf.R;
    kf.K = kf.S_p * kf.C * inv(kf.C * kf.S_p * transpose(kf.C) + kf.Q);
end
