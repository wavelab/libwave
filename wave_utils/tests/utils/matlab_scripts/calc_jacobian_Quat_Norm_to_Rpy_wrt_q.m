qr = sym('qr','real');
qx = sym('qx','real');
qy = sym('qy','real');
qz = sym('qz','real');

yaw = atan2(2 * (qr*qz + qx*qy), 1 - 2*(qy^2 + qz^2));
pitch = asin(2 * (qr*qy - qx*qz));
roll = atan2(2* (qr*qx + qy*qz), 1 - 2*(qx^2 + qy^2));

f = [yaw pitch roll]';
J1 = jacobian(f, [qr qx qy qz]);

% delta = 1/2
yaw = -2*atan2(qx,qr);
pitch = pi/2;
roll = 0;
f = [yaw pitch roll]';
J2 = jacobian(f, [qr qx qy qz]);

% delta = -1/2
yaw = 2*atan2(qx,qr);
pitch = -pi/2;
roll = 0;
f = [yaw pitch roll]';
J3 = jacobian(f, [qr qx qy qz]);


