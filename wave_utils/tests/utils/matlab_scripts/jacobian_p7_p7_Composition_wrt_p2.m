function J = jacobian_p7_p7_Composition_wrt_p2(p1, p2)

J = zeros(7,7);
qr1 = p1(4);
qx1 = p1(5);
qy1 = p1(6);
qz1 = p1(7);

J(4:7, 4:7) = [qr1, -qx1, -qy1, -qz1;
               qx1, qr1, -qz1, qy1;
               qy1, qz1, qr1, -qx1;
               qz1, -qy1, qx1, qr1];
J(4:7, 4:7) = J(4:7, 4:7) * jacobian_Quat_Norm_wrt_q(p2(4:7));

J(1:3, 1:3) = eye(3, 3);

qr = p1(4);
qx = p1(5);
qy = p1(6);
qz = p1(7);

J(1:3, 1:3) = [0.5 - qy*qy - qz*qz , qx*qy - qr*qz, qr*qy + qx*qz;
               qr*qz + qx*qy, 0.5 - qx*qx - qz*qz, qy*qz - qr*qx;
               qx*qz - qr*qy, qr*qx + qy*qz, 0.5 - qx*qx - qy*qy];
           
J(1:3, 1:3) = J(1:3, 1:3) * 2;
