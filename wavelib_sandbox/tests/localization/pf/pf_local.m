function [X] = pf_local(t, dt, meas, D, R, Q, d, X, Xp, mf, y, u)
    w = zeros(1, D);

    % sample
    for dd = 1:D
        e = gaussian_noise(R);
        Xp(:, dd) = [X(1, dd) + u(1, t) * cos(X(3, dd)) * dt;
                    X(2, dd) + u(1, t) * sin(X(3, dd)) * dt;
                    X(3, dd) + u(2, t) * dt] + e;
                
        switch(meas)
            case 1  % 1 - range
                r = range(mf(:, t), Xp(:, dd));
                hXp = r + d;
            case 2  % 2 - bearing
                b = bearing(mf(:, t), Xp(:, dd));
                hXp = b + d;
            case 3  % 3 - both
                r = range(mf(:, t), Xp(:, dd));
                b = bearing(mf(:, t), Xp(:, dd));
                hXp = [r; b] + d;
        end
        w(dd) = max(1e-8, mvnpdf(y(:, t), hXp, Q));
    end
    W = cumsum(w);

    % importance re-sample
    for dd = 1:D
        seed = max(W) * rand(1);
        find(W > seed, 1)
        X(:, dd) = Xp(:, find(W > seed, 1));
    end
end