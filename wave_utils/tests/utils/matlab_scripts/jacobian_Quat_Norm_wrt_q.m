function J = jacobian_Quat_Norm_wrt_q(q)

qr = q(1);
qx = q(2);
qy = q(3);
qz = q(4);

k = 1 / (qr * qr + qx * qx + qy * qy + qz * qz) ^ 1.5;

J = [qx * qx + qy * qy + qz * qz, -qr * qx, -qr * qy, -qr * qz;
     -qx * qr, qr * qr + qy * qy + qz * qz, -qx * qy, -qx * qz;
     -qy * qr, -qy * qx, qr * qr + qx * qx + qz * qz, -qy * qz;
     -qz * qr, -qz * qx, -qz * qy, qr * qr + qx * qx + qy * qy];

J = J * k;