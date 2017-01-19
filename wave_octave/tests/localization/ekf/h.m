function h_t = h(mf, y, mu_p, data)
    dx = mf(1) - mu_p(1);
    dy = mf(2) - mu_p(2);

    switch(data.meas)
        case 1
            I = y - sqrt(dx^2 + dy^2);
        case 2
            I = y - atan2(dy, dx) - mu_p(3);
            I = mod(I + pi, 2 * pi) - pi;
        case 3
            I = y - [
                sqrt(dx^2 + dy^2);
                atan2(dy, dx) - mu_p(3)
            ];
            I(2) = mod(I(2) + pi, 2 * pi) - pi;
    end
    
    h_t = I;
end