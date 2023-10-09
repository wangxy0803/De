%%
% Simulation
clc;
clear;
close all;

% 天蓝色
color_choose = [126/255 208/255 248/255];
% 棕色
color_choose = [191/255 168/255 147/255];
% 墨绿
color_choose = [62/255 132/255 140/255];
% 柴色
color_choose = [177/255 86/255 70/255];
% 紫色
color_choose = [126/255 162/255 237/255];
% 红色
dot_color = [215/255 99/255 100/255];


% set parameters
INVALID_VALUE = 1000000;
a1 = [160, 250];
a2 = [-160, 500];
a3 = [-160, 0];
% circumscribed circle radius of hexagon
r_hexagon = 60/cos(30/180*pi);
b1 = [r_hexagon*cos(60/180*pi), r_hexagon*sin(60/180*pi)];
b2 = [r_hexagon*cos(120/180*pi), r_hexagon*sin(120/180*pi)];
b3 = [r_hexagon*cos(180/180*pi), r_hexagon*sin(180/180*pi)];
b4 = [r_hexagon*cos(240/180*pi), r_hexagon*sin(240/180*pi)];
b5 = [r_hexagon*cos(300/180*pi), r_hexagon*sin(300/180*pi)];
b6 = [r_hexagon*cos(360/180*pi), r_hexagon*sin(360/180*pi)];
b11 = [50,0];
b22 = [-50,0];
distance = b11(1)-b22(1);
width = 320;
len = 1000;
a_coordinate_source = [0, 0];
r = 7;
rectangular_L = 60;
rectangular_W = 30;
leg_1 = 230;
leg_2 = 180;
leg_3 = leg_2;
% unique position distance gap
distance_L1 = 10;
% distance of ending leaving
distance_L2 = 80;


% set initial configuration
phi_initial_down = 0;
p_initial_down = [0, 0]';
R_initial_down = [cos(phi_initial_down/180*pi), -sin(phi_initial_down/180*pi);sin(phi_initial_down/180*pi), cos(phi_initial_down/180*pi)];

% set target configuration
phi_target_down = 40;
p_target_down = [55, 430]';
R_target_down = [cos(phi_target_down/180*pi), -sin(phi_target_down/180*pi);sin(phi_target_down/180*pi), cos(phi_target_down/180*pi)];

% get initial d and draw the initial configuration
p = p_initial_down;
phi = phi_initial_down;
R = R_initial_down;
disp('get initial d:');
[d_1x_down, d_1y_1_down, d_1y_2_down, ~, ~, ~] = calcaulate_d_3bar(p, phi/180*pi, b11, a1, 1/2*len, INVALID_VALUE, rectangular_L, leg_1);
disp('calcaulate_d of joint_1 end!');
[d_2x_down, d_2y_1_down, d_2y_2_down, ~, ~, ~] = calcaulate_d_3bar(p, phi/180*pi, b22, a2, 1/2*len, INVALID_VALUE, rectangular_L, leg_2);
disp('calcaulate_d of joint_2 end!');
[d_3x_down, d_3y_1_down, d_3y_2_down, ~, ~, ~] = calcaulate_d_3bar(p, phi/180*pi, b22, a3, 1/2*len, INVALID_VALUE, rectangular_L, leg_3);
disp('calcaulate_d of joint_3 end!');
fprintf('\n')

[d_1y_initial_down,d_2y_initial_down,d_3y_initial_down] = select_d(d_1y_1_down, d_1y_2_down, d_2y_1_down, d_2y_2_down, d_3y_1_down, d_3y_2_down, INVALID_VALUE, a2, a3, rectangular_L, 'configuration_1', 'up');
[TRUE_OR_FALSE, ~] = check_limitation(distance,R*b22'+p, a1(2),d_1y_initial_down,leg_1,width);
if TRUE_OR_FALSE
    draw_base_3bar(a_coordinate_source,'{b}',p,R,b1,b2,b3,b4,b5,b6,len,width,rectangular_W,phi,'change_color',color_choose,"coordinate",0);
    draw_move_3bar(d_1x_down,d_2x_down,d_3x_down,d_1y_initial_down,d_2y_initial_down,d_3y_initial_down,width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R,p,'change_color',color_choose);
else
    error('slider1 is out of the limitation!');
end







% --------------------------------------------------------------------------------------------------------------------------------------------------------
% upper action
p_final_down  = p;
R_final_down = [R(1,1), R(1,2), 0;
    R(2,1), R(2,2), 0;
    0, 0, 1;];
% set parameters
% the first rotation joint discribed in frame {b}
R_outer_circle_down = r_hexagon*0.8;
bbb1 = [R_outer_circle_down*cos(0/180*pi), R_outer_circle_down*sin(0/180*pi), 0]';
bbb2 = [R_outer_circle_down*cos(120/180*pi), R_outer_circle_down*sin(120/180*pi), 0]';
bbb3 = [R_outer_circle_down*cos(240/180*pi), R_outer_circle_down*sin(240/180*pi), 0]';
% configuration of the upper platform in frame {c}
c1 = [r_hexagon*1.2*cos(60/180*pi), r_hexagon*1.2*sin(60/180*pi), 0];
c2 = [r_hexagon*1.2*cos(120/180*pi), r_hexagon*1.2*sin(120/180*pi), 0];
c3 = [r_hexagon*1.2*cos(180/180*pi), r_hexagon*1.2*sin(180/180*pi), 0];
c4 = [r_hexagon*1.2*cos(240/180*pi), r_hexagon*1.2*sin(240/180*pi), 0];
c5 = [r_hexagon*1.2*cos(300/180*pi), r_hexagon*1.2*sin(300/180*pi), 0];
c6 = [r_hexagon*1.2*cos(360/180*pi), r_hexagon*1.2*sin(360/180*pi), 0];
% the second rotation joint on the upper platform in frame {c}
R_outer_circle_up = 60;
c11 = [R_outer_circle_up*cos(0/180*pi), R_outer_circle_up*sin(0/180*pi), 0]';
c22 = [R_outer_circle_up*cos(120/180*pi), R_outer_circle_up*sin(120/180*pi), 0]';
c33 = [R_outer_circle_up*cos(240/180*pi), R_outer_circle_up*sin(240/180*pi), 0]';
b_coordinate_source = [a_coordinate_source(1)+p_final_down(1),a_coordinate_source(2)+p_final_down(2),0];
% set length of legs
leg11 = 200;
leg12 = 200;
leg21 = 200;
leg22 = 200;
leg31 = 200;
leg32 = 200;

% set initial configuration in frame {b}
% *************************************
phi_initial_x_up = 0/180*pi;  % DOF_1
phi_initial_y_up = 0/180*pi;  % DOF_2
p_initial_z_up = -150;  % DOF_3
% *************************************
[R_initial_up, p_initial_up, phi_initial_up] = upper_inverse_kinematic_configuration(phi_initial_x_up, phi_initial_y_up, p_initial_z_up, R_outer_circle_up);

p_all = [];
range_z = 0:5:400;
range_phi_x = -pi/2:0.01:pi/2;
range_phi_y = -pi/2:0.01:pi/2;
COUNT1 = length(range_z);
COUNT2 = length(range_phi_x);
COUNT3 = length(range_phi_y);
i = 1;
j = 1;
k = 1;

while i <= COUNT1
    while j <= COUNT2
        while k <= COUNT3
            z = range_z(i);
            phix = 30/180*pi;
            phiy = 90/180*pi;
            [R, p, phi] = upper_inverse_kinematic_configuration(phix, phiy, z, R_outer_circle_up)

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

            if isempty(sol_1.theta1) || isempty(sol_2.theta2) || isempty(sol_3.theta3)
                continue;
            end
            
            num_1 = 1;
            num_2 = 1;
            num_3 = 1;
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


            double(sol_1.theta1(num_1))/pi*180
            double(sol_2.theta2(num_2))/pi*180
            double(sol_3.theta3(num_3))/pi*180
            p_all = [p_all, p];
            Representative_dot = b_coordinate_source + R*p;
            draw_coordinate_system(100,R,[a_coordinate_source(1)+100,a_coordinate_source(2),0]','rgb',"A");
            plot(Representative_dot(1), Representative_dot(2), '.');
            k = k+1;
        end
        j = j+1;
    end
    i = i+1;
end


