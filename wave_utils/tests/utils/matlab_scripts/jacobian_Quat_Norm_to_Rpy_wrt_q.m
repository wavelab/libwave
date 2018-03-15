function J =   jacobian_Quat_Norm_to_Rpy_wrt_q(q)

qr = q(1);
qx = q(2);
qy = q(3);
qz = q(4);

delta = qr * qy - qx * qz;

if (abs(delta - 0.5) < 1e-10)
                                    
    J = [(2*qx)/(qr*qr + qx*qx), -(2*qr)/(qr*qr + qx*qx), 0, 0;
                              0,                       0, 0, 0;
                              0,                       0, 0, 0];


elseif (abs(delta + 0.5) < 1e-10)

    J= [-(2*qx)/(qr*qr + qx*qx), (2*qr)/(qr*qr + qx*qx), 0, 0;
                              0,                      0, 0, 0;
                              0,                      0, 0, 0];
else 
      
    sq = @(x) x * x;


    
    J = [2*qz*(-2*qy*qy - 2*qz*qz + 1)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)), ...
        2*qy*(-2*qy*qy - 2*qz*qz + 1)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)), ...
        2*qx*(-2*qy*qy - 2*qz*qz + 1)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)) - 4*qy*(-2*qr*qz - 2*qx*qy)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)), ...
        2*qr*(-2*qy*qy - 2*qz*qz + 1)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)) - 4*qz*(-2*qr*qz - 2*qx*qy)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1));

        2*qy/sqrt(-sq(2*qr*qy - 2*qx*qz) + 1), ...
        -2*qz/sqrt(-sq(2*qr*qy - 2*qx*qz) + 1), ...
        2*qr/sqrt(-sq(2*qr*qy - 2*qx*qz) + 1), ...
        -2*qx/sqrt(-sq(2*qr*qy - 2*qx*qz) + 1);

        2*qx*(-2*qx*qx - 2*qy*qy + 1)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1)), ...
        2*qr*(-2*qx*qx - 2*qy*qy + 1)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1)) - 4*qx*(-2*qr*qx - 2*qy*qz)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1)), ...
        -4*qy*(-2*qr*qx - 2*qy*qz)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1)) + 2*qz*(-2*qx*qx - 2*qy*qy + 1)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1)), ...
        2*qy*(-2*qx*qx - 2*qy*qy + 1)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1))];
    
end


