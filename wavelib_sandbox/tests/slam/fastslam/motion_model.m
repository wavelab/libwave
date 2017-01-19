function x_t = motion_model(x, u, dt)
    x_t = [
        x(1) + u(1) * cos(x(3)) * dt;
        x(2) + u(1) * sin(x(3)) * dt;
        x(3) + u(2) * dt
    ];
end

