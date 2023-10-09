function [sol_1, sol_2, sol_3, num_1, num_2, num_3] = upper_inverse_kinematic_solve_equations(p, R, R_outer_circle_down, R_outer_circle_up, leg11, leg12, leg21, leg22, leg31, leg32)
% rotation angles of active joints 
syms theta1 theta2 theta3;
% rotation angles of passive joints 
syms theta4 theta5 theta6;

% X of closed chain space vector
eqn_1 = leg12*cos(theta4)*cos(0/180*pi) == p(1)+R_outer_circle_up*R(1,1) - cos(0/180*pi)*(R_outer_circle_down+leg11*cos(theta1));
eqn_2 = leg22*cos(theta5)*cos(120/180*pi) == p(1)-1/2*R_outer_circle_up*R(1,1)+sqrt(3)/2*R_outer_circle_up*R(1,2) - cos(120/180*pi)*(R_outer_circle_down+leg21*cos(theta2));
eqn_3 = leg32*cos(theta6)*cos(240/180*pi) == p(1)-1/2*R_outer_circle_up*R(1,1)-sqrt(3)/2*R_outer_circle_up*R(1,2) - cos(240/180*pi)*(R_outer_circle_down+leg31*cos(theta3));

% Y of closed chain space vector
% eqn_4 = leg12*cos(theta4)*sin(0/180*pi) == p(2)+R_outer_circle_up*R(2,1) - sin(0/180*pi)*(R_outer_circle_down+leg11*cos(theta1));
% eqn_5 = leg22*cos(theta5)*sin(120/180*pi) == p(2)-1/2*R_outer_circle_up*R(2,1)+sqrt(3)/2*R_outer_circle_up*R(2,2) - sin(120/180*pi)*(R_outer_circle_down+leg21*cos(theta2));
% eqn_6 = leg32*cos(theta6)*sin(240/180*pi) == p(2)-1/2*R_outer_circle_up*R(2,1)-sqrt(3)/2*R_outer_circle_up*R(2,2) - sin(240/180*pi)*(R_outer_circle_down+leg31*cos(theta3));

% Z of closed chain space vector
eqn_7 = leg12*sin(theta4) == -(p(3)+R_outer_circle_up*R(3,1)) - leg11*sin(theta1);
eqn_8 = leg22*sin(theta5) == -(p(3)-1/2*R_outer_circle_up*R(3,1)+sqrt(3)/2*R_outer_circle_up*R(3,2)) - leg21*sin(theta2);
eqn_9 = leg32*sin(theta6) == -(p(3)-1/2*R_outer_circle_up*R(3,1)-sqrt(3)/2*R_outer_circle_up*R(3,2)) - leg31*sin(theta3);

% combine X and Z
eqn_10 = (eqn_7*cos(0/180*pi))^2 + (eqn_1)^2;
eqn_11 = (eqn_8*cos(120/180*pi))^2 + (eqn_2)^2;
eqn_12 = (eqn_9*cos(240/180*pi))^2 + (eqn_3)^2;


% use solve function to get analytical solutions
sol_1 = solve(eqn_1, eqn_7, eqn_10, theta1, theta4);
sol_2 = solve(eqn_2, eqn_8, eqn_11, theta2, theta5);
sol_3 = solve(eqn_3, eqn_9, eqn_12, theta3, theta6);

% use vpasolve function to get numerical solutions
if isempty(sol_1.theta1)
sol_1 = vpasolve(eqn_1, eqn_7, eqn_10, theta1, theta4);
end

if isempty(sol_2.theta2)
sol_2 = vpasolve(eqn_2, eqn_8, eqn_11, theta2, theta5);
end

if isempty(sol_3.theta3)
sol_3 = vpasolve(eqn_3, eqn_9, eqn_12, theta3, theta6);
end

% choose the outward protrusion
num_1 = 1;
num_2 = 1;
num_3 = 1;
% r11 = real(double([leg11*cos(sol_1.theta1(num_1)), 0, -leg11*sin(sol_1.theta1(num_1))]'));
% r21 = real(double([-1/2*leg21*cos(sol_2.theta2(num_2)), sqrt(3)/2*leg21*cos(sol_2.theta2(num_2)), -leg21*sin(sol_2.theta2(num_2))]'));
% r31 = real(double([-1/2*leg31*cos(sol_3.theta3(num_3)), -sqrt(3)/2*leg31*cos(sol_3.theta3(num_3)), -leg31*sin(sol_3.theta3(num_3))]'));
r11 = double([leg11*cos(sol_1.theta1(num_1)), 0, -leg11*sin(sol_1.theta1(num_1))]');
r21 = double([-1/2*leg21*cos(sol_2.theta2(num_2)), sqrt(3)/2*leg21*cos(sol_2.theta2(num_2)), -leg21*sin(sol_2.theta2(num_2))]');
r31 = double([-1/2*leg31*cos(sol_3.theta3(num_3)), -sqrt(3)/2*leg31*cos(sol_3.theta3(num_3)), -leg31*sin(sol_3.theta3(num_3))]');
if abs(atan2(r11(2), r11(1)) - 0/180*pi) > 0.001
num_1 = 2;
end
if abs(atan2(r21(2), r21(1)) - 120/180*pi) > 0.001
num_2 = 2;
end
if abs(atan2(r31(2), r31(1)) - -120/180*pi) > 0.001
num_3 = 2;
end

end