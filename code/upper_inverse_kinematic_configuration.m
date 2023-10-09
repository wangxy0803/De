function [R, p, phi] = upper_inverse_kinematic_configuration(phi_x, phi_y, p_z, R_outer_circle_up)
phi_z = atan2(-sin(phi_x)*sin(phi_y),(cos(phi_x)+cos(phi_y)));
phi = [phi_x, phi_y, phi_z];
Rx = [1, 0, 0; 
0, cos(phi(1)), -sin(phi(1)); 
0, sin(phi(1)), cos(phi(1));];
Ry = [cos(phi(2)), 0, sin(phi(2)); 
        0, 1, 0; 
        -sin(phi(2)), 0, cos(phi(2));];
Rz = [cos(phi(3)), -sin(phi(3)), 0; 
        sin(phi(3)), cos(phi(3)), 0; 
        0, 0, 1;];
R = Rx*Ry*Rz;

p_x = R_outer_circle_up*(R(1,1)-R(2,2))/2;
p_y = -R(2,1)*R_outer_circle_up;
p = [p_x, p_y, p_z]';
end