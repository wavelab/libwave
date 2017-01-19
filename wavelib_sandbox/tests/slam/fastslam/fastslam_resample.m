function [X, mu, S] = fastslam_resample(w, Xp, mup, Sp, n, M, D)
    W = cumsum(w);
    X = zeros(n, D);  % Vehicle states
    mu = zeros(2, M, D);  % Feature means
    S = zeros(2, 2, M, D); % Feature covariances

    for d = 1:D
        seed = W(end) * rand(1);
        cur = find(W > seed, 1);
        X(:, d) = Xp(:, cur);
        mu(:, :, d) = mup(:, :, cur);
        S(:, :, :, d) = Sp(:, :, :, cur);
    end
end