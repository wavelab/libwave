function [x, u, Jx, Ju, P_S, K] = lqr_sim(Ad, Bd, Q, R, t0, tf, dt, x0, ss)
% Runs an lqr simulation from t0 to tf with the system defined by Ad, Bd,Q
% and R

    % Costate setup
    T = t0:dt:tf;
    P = zeros(3, 3, length(T));
    P_S(:, :, length(T)) = Q;
    Pn = P_S(:, :, length(T));

    % Solve for costate
    for t = length(T)-1:-1:1
        P = Q + Ad' * Pn * Ad - Ad' * Pn * Bd * inv(Bd' * Pn * Bd + R) * Bd' * Pn * Ad;
        P_S(:,:,t) = P;
        Pn = P;
    end

    % Setup storage structures
    x = zeros(3, length(T));
    x(:, 1) = x0';
    u = zeros(1, length(T) - 1);
    Jx = 0;
    Ju = 0;

    % Steady state comparison
    if (ss)
        Kss = dlqr(Ad, Bd, Q, R);
    end

    % Solve for control gain and simulate
    for t=1:length(T)-1
        K = inv(Bd' * P_S(:, :, t+ 1) * Bd + R) * Bd' * P_S(:, :, t + 1) * Ad;
        u(:, t) = -K * x(:, t);
        if(ss)
            u(:,t) = -Kss * x(:, t);
        end
        x(:, t + 1) = Ad * x(:, t) + Bd * u(:, t);

        Jx = Jx + 1/2 * x(:, t)' * x(:, t);
        Ju = Ju + 1/2 * u(:, t)' * u(:, t);
    end

end