function J = jacobian_p6_to_p7_wrt_p(p)


roll = p(6);
pitch = p(5);
yaw = p(4);

ccc = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2);
ccs = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
csc = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
scs = sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
css = cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
scc = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2);
ssc = sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
sss = sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);

J = eye(7,6);
J(4:7,4:6) = [(ssc - ccs) / 2.0, (scs - csc) / 2.0, (css - scc) / 2.0;
             -(csc + scs) / 2.0, -(ssc + ccs) / 2.0, (ccc + sss) / 2.0;
              (scc - css) / 2.0, (ccc - sss) / 2.0, (ccs - ssc) / 2.0;
              (ccc + sss) / 2.0, -(css + scc) / 2.0, -(csc + scs) / 2.0];