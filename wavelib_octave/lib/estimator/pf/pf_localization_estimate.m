function pf = pf_localization_estimate(pf, Xp_func, hXp_func, mf, u, y)
    % sampling
    for m = 1:pf.M
        Xp = feval(Xp_func, u, R);
        hXp = feval(hXp_func, mf, Xp);

        pf.Xp(:, m) = Xp;
        pf.w(m) = max(1e-8, mvnpdf(y, hXp, pf.Q));
    end

    % importance resampling
    W = cumsum(pf.w);
    for m = 1:pf.M
        seed = W(end) * rand(1);
        pf.X(m) = pf.Xp(find(W > seed, 1));
    end

    % record mean particle
    pf.mu = mean(pf.X);
    pf.S = var(pf.X);
end
