function J = jacobian_p7_p7_Composition_wrt_p1(p1, p2)

J = zeros(7,7);
qr2 = p2(4);
qx2 = p2(5);
qy2 = p2(6);
qz2 = p2(7);

J(4:7, 4:7) = [qr2, -qx2, -qy2, -qz2;
               qx2, qr2, qz2, -qy2;
               qy2, -qz2, qr2, qx2;
               qz2, qy2, -qx2, qr2];
J(4:7, 4:7) = J(4:7, 4:7) * jacobian_Quat_Norm_wrt_q(p1(4:7));

J(1:3, 1:3) = eye(3, 3);

ax = p2(1); 
ay = p2(2);
az = p2(3);

qr = p1(4);
qx = p1(5);
qy = p1(6);
qz = p1(7);

J(1:3, 4:7) = [-qz*ay+qy*az, qy*ay+qz*az, -2*qy*ax+qx*ay+qr*az, -2*qz*ax-qr*ay+qx*az;
               qz*ax-qx*az, qy*ax-2*qx*ay-qr*az, qx*ax+qz*az, qr*ax-2*qz*ay+qy*az;
               -qy*ax+qx*ay, qz*ax+qr*ay-2*qx*az, -qr*ax+qz*ay-2*qy*az, qx*ax+qy*ay];
           
J(1:3, 4:7) = J(1:3, 4:7) * 2;
J(1:3, 4:7) = J(1:3, 4:7) * jacobian_Quat_Norm_wrt_q(p1(4:7));
