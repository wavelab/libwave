function [mup_update, Sp_update, dx, dy, rp] = feature_update(Xp, mu, y, Qi, S)
    % Predicted range
    dx = mu(1) - Xp(1);
    dy = mu(2) - Xp(2);
    rp = sqrt(dx^2 + dy^2);

    % Calculate Jacobian
    Ht = [ 
        dx / rp, dy / rp; 
        -dy / rp^2, dx / rp^2
    ];

    % Calculate Innovation
    I = y - [
        rp; 
        mod(atan2(dy, dx) - Xp(3) + pi, 2 * pi) - pi
    ];

    % Measurement update
    Q = Ht * S * transpose(Ht) + Qi;
    K = S * transpose(Ht) / Q;
    mup_update = mu + K * I;
    Sp_update = (eye(2) - K * Ht) * S;
end

