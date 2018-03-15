function v = quatToRpy(q)
    v = zeros(3, 1);

    qr = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);

    delta = qr * qy - qx * qz;

    if (abs(delta - 0.5) < 1e-6)
        yaw = -2 * atan2(qx, qr);
        pitch = pi / 2;
        roll = 0;

    elseif (abs(delta + 0.5) < 1e-6)
        yaw = 2 * atan2(qx, qr);
        pitch = -pi / 2;
        roll = 0;
    else
        yaw = atan2(2 * (qr * qz + qx * qy), (1 - 2 * (qy * qy + qz * qz)));
        pitch = asin(2 * delta);
        roll = atan2(2 * (qr * qx + qy * qz), (1 - 2 * (qx * qx + qy * qy)));
    end

    v(1) = yaw;
    v(2) = pitch;
    v(3) = roll;
end