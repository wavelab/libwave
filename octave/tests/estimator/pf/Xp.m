function Xp_m = Xp(X, u, R)
    A = 0.8;
    B = 3;

    e = gaussian_noise(R);
    Xp_m = A * X + B * u + e;
end