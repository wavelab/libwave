function g_t = g(u, x, dt, data)
    g_t = [ ...
        x(1) + u(1) * cos(x(3)) * dt;
        x(2) + u(1) * sin(x(3)) * dt;
        x(3) + ((u(1) * tan(x(3))) / data.L);
    ];
end