function kf = kf_estimate(kf, u, y)
    kf = kf_prediction_update(kf, u);
    kf = kf_measurement_update(kf, y);
end
