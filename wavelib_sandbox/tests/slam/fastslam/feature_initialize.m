function [mup_init, Sp_init, dx, dy, rp] = feature_initialize(Xp, y, Qi)
    % Feature initialization
    mup_init(1) = Xp(1) + y(1) * cos(y(2) + Xp(3));
    mup_init(2) = Xp(2) + y(1) * sin(y(2) + Xp(3));

    % Predicted range
    dx = mup_init(1) - Xp(1);
    dy = mup_init(2) - Xp(2);
    rp = sqrt((dx)^2 + (dy)^2);

    % Calculate Jacobian
    Ht = [ 
        dx / rp, dy / rp; 
        -dy / rp^2, dx / rp^2
    ];
    Sp_init = Ht \ Qi * transpose(inv(Ht));
end

