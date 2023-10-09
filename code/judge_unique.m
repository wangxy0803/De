function Unique = judge_unique(alpha1, alpha2, alpha3, R_outer_circle_down, R_outer_circle_up, ...
    leg11, leg21, leg31, theta1, theta2, theta3, sphere1, sphere2, sphere3, phi1, phi2, phi3)

E_1 = 2*leg11*cos(alpha1)*(cos(alpha1)*cos(theta1)*sphere1(3) + sin(theta1)*(-R_outer_circle_down*cos(alpha1) + sphere1(1)));
E_2 = 2*leg21*cos(alpha2)*(cos(alpha2)*cos(theta2)*sphere2(3) + sin(theta2)*(-R_outer_circle_down*cos(alpha2) + sphere2(1)));
E_3 = 2*leg31*cos(alpha3)*(cos(alpha3)*cos(theta3)*sphere3(3) + sin(theta3)*(-R_outer_circle_down*cos(alpha3) + sphere3(1)));

D_1 = [-2*cos(alpha1)*(R_outer_circle_down + leg11*cos(theta1)) + 2*sphere1(1), 0, 2*cos(alpha1)^2*(sphere1(3) + leg11*sin(theta1))];
D_2 = [-2*cos(alpha2)*(R_outer_circle_down + leg21*cos(theta2)) + 2*sphere2(1), 0, 2*cos(alpha2)^2*(sphere2(3) + leg21*sin(theta2))];
D_3 = [-2*cos(alpha3)*(R_outer_circle_down + leg31*cos(theta3)) + 2*sphere3(1), 0, 2*cos(alpha3)^2*(sphere3(3) + leg31*sin(theta3))];
J_s = [D_1, zeros(1, 3), zeros(1, 3); zeros(1, 3), D_2, zeros(1, 3); zeros(1, 3), zeros(1, 3), D_3;];
J_t_sphere1 = [1, 0, 0, 0, -R_outer_circle_up*sin(phi2)*cos(alpha1+phi3), -R_outer_circle_up*cos(phi2)*sin(alpha1 + phi3);
    0, 1, 0, R_outer_circle_up*(cos(phi1)*sin(phi2)*cos(alpha1 + phi3) - sin(phi1)*sin(alpha1 + phi3)), R_outer_circle_up*sin(phi1)*cos(phi2)*cos(alpha1 + phi3), R_outer_circle_up*(phi1*cos(alpha1+phi3) - sin(phi1)*sin(phi2)*sin(alpha1 + phi3));
    0, 0, 1, R_outer_circle_up*(sin(phi1)*sin(phi2)*cos(alpha1 + phi3) + cos(phi1)*sin(alpha1 + phi3)), -R_outer_circle_up*cos(phi1)*cos(phi2)*cos(alpha1 + phi3), R_outer_circle_up*(sin(phi1)*cos(alpha1 + phi3) + cos(phi1)*sin(phi2)*sin(alpha1 + phi3))];
J_t_sphere2 = [1, 0, 0, 0, -R_outer_circle_up*sin(phi2)*cos(alpha2+phi3), -R_outer_circle_up*cos(phi2)*sin(alpha2 + phi3);
    0, 1, 0, R_outer_circle_up*(cos(phi1)*sin(phi2)*cos(alpha2 + phi3) - sin(phi1)*sin(alpha2 + phi3)), R_outer_circle_up*sin(phi1)*cos(phi2)*cos(alpha2 + phi3), R_outer_circle_up*(phi1*cos(alpha2+phi3) - sin(phi1)*sin(phi2)*sin(alpha2 + phi3));
    0, 0, 1, R_outer_circle_up*(sin(phi1)*sin(phi2)*cos(alpha2 + phi3) + cos(phi1)*sin(alpha2 + phi3)), -R_outer_circle_up*cos(phi1)*cos(phi2)*cos(alpha2 + phi3), R_outer_circle_up*(sin(phi1)*cos(alpha2 + phi3) + cos(phi1)*sin(phi2)*sin(alpha2 + phi3))];
J_t_sphere3 = [1, 0, 0, 0, -R_outer_circle_up*sin(phi2)*cos(alpha3+phi3), -R_outer_circle_up*cos(phi2)*sin(alpha2 + phi3);
    0, 1, 0, R_outer_circle_up*(cos(phi1)*sin(phi2)*cos(alpha3 + phi3) - sin(phi1)*sin(alpha3 + phi3)), R_outer_circle_up*sin(phi1)*cos(phi2)*cos(alpha3 + phi3), R_outer_circle_up*(phi1*cos(alpha3+phi3) - sin(phi1)*sin(phi2)*sin(alpha3+ phi3));
    0, 0, 1, R_outer_circle_up*(sin(phi1)*sin(phi2)*cos(alpha3 + phi3) + cos(phi1)*sin(alpha3+ phi3)), -R_outer_circle_up*cos(phi1)*cos(phi2)*cos(alpha3 + phi3), R_outer_circle_up*(sin(phi1)*cos(alpha3 + phi3) + cos(phi1)*sin(phi2)*sin(alpha3 + phi3))];
J_ts = [J_t_sphere1; J_t_sphere2; J_t_sphere3];
J_p_x = [0, -R_outer_circle_up*cos(phi2)*sin(phi2)*(cos(phi1) + cos(phi2)*sin(phi1)*sin(phi2))/(2*(1 + cos(phi1)*cos(phi2))^2), -R_outer_circle_up*cos(phi2)*sin(phi1)*(cos(phi1) + cos(phi2) + sin(phi1)*sin(phi2))];
J_p_y = [0, -R_outer_circle_up*cos(phi2)*sin(phi2)*(cos(phi1) + cos(phi2))/(1 + cos(phi1)*cos(phi2))^2, R_outer_circle_up*sin(phi1)*(sin(phi1)^2*sin(phi2)^4 - cos(phi2)^4 - cos(phi1)^2*cos(2*phi2) - 1/4*cos(phi1)*(5*cos(phi2) + 3*cos(3*phi2)))/(1+cos(phi1)*cos(phi2))^3];
J_phi_z = [0, -sin(phi2)/(1 + cos(phi1)*cos(phi2)), -sin(phi1)/(1 + cos(phi1)*cos(phi2))];
J_xt = [J_p_x; J_p_y; eye(3,3); J_phi_z];

J1 = J_s*J_ts*J_xt;
J2 = [E_1, 0, 0;
    0, E_2, 0;
    0, 0, E_3];

Unique = false;
if det(J1) ==0 || det(J2) ==0
    Unique = true;
end

end