%%
% Simulation
clc;
clear;
close all;

% 调色
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

% /**************************************下层机器人********************************************/
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

% [TRUE_OR_FALSE, ~] = check_limitation(distance,R*b22'+p, a1(2),d_1y_initial_down,leg_1,width);
% if TRUE_OR_FALSE
%     draw_base_3bar(a_coordinate_source,'{b}',p,R,b1,b2,b3,b4,b5,b6,len,width,rectangular_W,phi,'change_color',color_choose,"coordinate",0);
%     draw_move_3bar(d_1x_down,d_2x_down,d_3x_down,d_1y_initial_down,d_2y_initial_down,d_3y_initial_down,width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R,p,'change_color',color_choose);
% else
%     error('slider1 is out of the limitation!');
% end


% draw skull
hold on
TR = stlread('skull.stl');
P = TR.Points;% vertices information
P = P*+30;
P(:,1)=P(:,1)+20;
P(:,2)=P(:,2)+180;
P(:,3)=P(:,3)-300;
CL = TR.ConnectivityList;% connection list
patch('vertices', P, 'faces', CL, 'facevertexcdata',P(:,3), 'facecolor', 'interp', 'edgecolor', 'none');%以z方向坐标作为颜色画出立体图
colormap("bone");view(-30,40);axis('equal');
% 
light('Position',[-70000 50000 1000000000],'Style','local');
% light('Position',[-70000 10000 1000000000],'Style','local');
% light('Position',[-70000 -50000 1000000000],'Style','local');
% light('Position',[70000 50000 -1000000000],'Style','local');
% light('Position',[70000 10000 -1000000000],'Style','local');
% light('Position',[70000 -50000 -1000000000],'Style','local');


% bottom robot's motion
% 这三个是用某个程序跑好的正运动学
load('p_actual.mat');
load('phi_actual.mat');
load('R_actual.mat');
% set trace of inverse kinematics
Tx = 0:0.01:5;
px_expect = [11*Tx, 55*ones([1,45*100])];
Ty = 0:0.01:50;
py_expect = 43/5*Ty;
Tphi = 0:0.01:50;
phi_expect = 4/5*Tphi;

d_1y_expect = [];
d_2y_expect = [];
d_3y_expect = [];
d_1x_expect = [];
d_2x_expect = [];
d_3x_expect = [];

for i = 1:length(Tphi)
    p = [px_expect(i), py_expect(i)]';
    phi = phi_expect(i);
    R = [cos(phi_expect(i)/180*pi), -sin(phi_expect(i)/180*pi);sin(phi_expect(i)/180*pi), cos(phi_expect(i)/180*pi)];
    
    [d_1x_down, d_1y_1_down, d_1y_2_down, ~, ~, ~] = calcaulate_d_3bar(p, phi/180*pi, b11, a1, 1/2*len, INVALID_VALUE, rectangular_L, leg_1);
    [d_2x_down, d_2y_1_down, d_2y_2_down, ~, ~, ~] = calcaulate_d_3bar(p, phi/180*pi, b22, a2, 1/2*len, INVALID_VALUE, rectangular_L, leg_2);
    [d_3x_down, d_3y_1_down, d_3y_2_down, ~, ~, ~] = calcaulate_d_3bar(p, phi/180*pi, b22, a3, 1/2*len, INVALID_VALUE, rectangular_L, leg_3);

    % 底层机器人逆运动学也有多解（两个解）问题，但比较容易排除，select_d是关于如何排除多解的函数
    [d_1y_down,d_2y_down,d_3y_down] = select_d(d_1y_1_down, d_1y_2_down, d_2y_1_down, d_2y_2_down, d_3y_1_down, d_3y_2_down, INVALID_VALUE, a2, a3, rectangular_L, 'configuration_1', 'up');
    % 判断滑块是否滑出轨道范围
    [TRUE_OR_FALSE, ~] = check_limitation(distance,R*b22'+p, a1(2),d_1y_down,leg_1,width);
    if TRUE_OR_FALSE
        d_1y_expect = [d_1y_expect, d_1y_down]; 
        d_1x_expect = [d_1x_expect, d_1x_down]; 
        d_2y_expect = [d_2y_expect, d_2y_down]; 
        d_2x_expect = [d_2x_expect, d_2x_down];
        d_3y_expect = [d_3y_expect, d_3y_down]; 
        d_3x_expect = [d_3x_expect, d_3x_down];

        % 画图
%         if mod(i, 100) == 0
%             traceA = p+R*b11';
%             traceB = p+R*b22';
%             plot(traceA(1),traceA(2),    ...
%             'Marker','o', ...
%             'MarkerSize',2,...
%             'MarkerEdgeColor',dot_color,...
%             'MarkerFaceColor',dot_color);
%             plot(traceB(1),traceB(2), ...
%             'Marker','o', ...
%             'MarkerSize',2,...
%             'MarkerEdgeColor',dot_color,...
%             'MarkerFaceColor',dot_color);
%         end

%         if mod(i, 100) == 0
%                 traceA = p_actual(:,i)+R_actual(:,(2*i-1):(2*i))*b11';
%                 traceB = p_actual(:,i)+R_actual(:,(2*i-1):(2*i))*b22';
%                 plot(traceA(1),traceA(2),    ...
%                 'Marker','o', ...
%                 'MarkerSize',1,...
%                 'MarkerEdgeColor','b',...
%                 'MarkerFaceColor','b');
%                 plot(traceB(1),traceB(2), ...
%                 'Marker','o', ...
%                 'MarkerSize',1,...
%                 'MarkerEdgeColor','b',...
%                 'MarkerFaceColor','b');
%         end

%         if  i == 2500
%             draw_base_3bar(a_coordinate_source,'{b}',p_actual(:,i),R_actual(:,(2*i-1):(2*i)),b1,b2,b3,b4,b5,b6,len,width,rectangular_W,phi_actual(i),'change_color',color_choose, "coordinate", 0);
%             draw_move_3bar(d_1x_down,d_2x_down,d_3x_down,d_1y_down,d_2y_down,d_3y_down,width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R_actual(:,(2*i-1):(2*i)),p_actual(:,i),'change_color',color_choose);
%         end

        if  i == length(Tphi)
            draw_base_3bar(a_coordinate_source,'{b}',p_actual(:,i),R_actual(:,(2*i-1):(2*i)),b1,b2,b3,b4,b5,b6,len,width,rectangular_W,phi_actual(i),'change_color',color_choose, "coordinate", 0,'transparency',0.3);
%             draw_move_3bar(d_1x_down,d_2x_down,d_3x_down,d_1y_down,d_2y_down,d_3y_down,width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R_actual(:,(2*i-1):(2*i)),p_actual(:,i),'change_color',color_choose,'transparency',0.3);
        end

    else
        error('slider1 is out of the limitation!');
    end
end



% /**************************************上层机器人********************************************/
% upper robot's motion
p_final_down  = p_actual(:,length(Tphi));
R_final_down = [R_actual(1,(2*length(Tphi)-1):(2*length(Tphi))),0;
                R_actual(2,(2*length(Tphi)-1):(2*length(Tphi))),0;
                0,0,1];

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


% ************************************************************************************************************************
% set target configuration
% *************************************
phi_target_x_down = -40/180*pi;  % DOF_1
phi_target_y_down = 30/180*pi;  % DOF_2
p_target_z_down = -220;  % DOF_3
% *************************************
[R_target_up, p_target_up, phi_target_up] = upper_inverse_kinematic_configuration(phi_target_x_down, phi_target_y_down, p_target_z_down, R_outer_circle_up);
% ************************************************************************************************************************



% get initital r and draw the initial configuration of 3D
% calculate the second rotation joint on the middle in frame {b}
p = p_initial_up;
R = R_initial_up;
[sol_1_initial_up, sol_2_initial_up, sol_3_initial_up, num_1_initial_up, num_2_initial_up, num_3_initial_up] = upper_inverse_kinematic_solve_equations(...
    p, R, R_outer_circle_down, R_outer_circle_up, leg11, leg12, leg21, leg22, leg31, leg32);

% set configuration to trace in frame {b}  
t = 0:0.4:500;
s = 0.005*t - 0.1*sin(0.2*pi*t)/pi;
phi_exp_x_up = -4/45*pi*s;
phi_exp_y_up = pi/15*s;
p_exp_z_up = -28*s - 150;



% maximum iterations
COUNT = size(t);
COUNT = COUNT(2);
% % current iterations
% count = 1;

% r11_exp = [];
% r21_exp = [];
% r31_exp = [];
% r11_a_exp = [];
% r21_a_exp = [];
% r31_a_exp = [];

% % get active joints's angles
% while count <= COUNT
%     
%     % *************************************
%     phi_new_exp_x = phi_exp_x_up(count);  % DOF_1
%     phi_new_exp_y = phi_exp_y_up(count);  % DOF_2
%     p_new_exp_z = p_exp_z_up(count);  % DOF_3
%     % *************************************
%     [R_new_exp_up, p_new_exp_up, phi_new_exp_up] = upper_inverse_kinematic_configuration(phi_new_exp_x, phi_new_exp_y, p_new_exp_z, R_outer_circle_up);
% 
%     % get initital r and draw the initial configuration of 3D
%     % calculate the second rotation joint on the middle in frame {b}
%     p = p_new_exp_up;
%     R = R_new_exp_up;
%     [sol_1_new_exp_up, sol_2_new_exp_up, sol_3_new_exp_up, num_1_new_exp_up, num_2_new_exp_up, num_3_new_exp_up] = upper_inverse_kinematic_solve_equations(...
%     p, R, R_outer_circle_down, R_outer_circle_up, leg11, leg12, leg21, leg22, leg31, leg32);
%     
%     % r in frame {b}
%     r11 = real(double([leg11*cos(sol_1_new_exp_up.theta1(num_1_new_exp_up)), 0, -leg11*sin(sol_1_new_exp_up.theta1(num_1_new_exp_up))]'));
%     r12 = real(double([leg12*cos(sol_1_new_exp_up.theta4(num_1_new_exp_up)), 0, -leg12*sin(sol_1_new_exp_up.theta4(num_1_new_exp_up))]'));
%     r21 = real(double([-1/2*leg21*cos(sol_2_new_exp_up.theta2(num_2_new_exp_up)), sqrt(3)/2*leg21*cos(sol_2_new_exp_up.theta2(num_2_new_exp_up)), -leg21*sin(sol_2_new_exp_up.theta2(num_2_new_exp_up))]'));
%     r22 = real(double([-1/2*leg22*cos(sol_2_new_exp_up.theta5(num_2_new_exp_up)), sqrt(3)/2*leg22*cos(sol_2_new_exp_up.theta5(num_2_new_exp_up)), -leg22*sin(sol_2_new_exp_up.theta5(num_2_new_exp_up))]'));
%     r31 = real(double([-1/2*leg31*cos(sol_3_new_exp_up.theta3(num_3_new_exp_up)), -sqrt(3)/2*leg31*cos(sol_3_new_exp_up.theta3(num_3_new_exp_up)), -leg31*sin(sol_3_new_exp_up.theta3(num_3_new_exp_up))]'));
%     r32 = real(double([-1/2*leg32*cos(sol_3_new_exp_up.theta6(num_3_new_exp_up)), -sqrt(3)/2*leg32*cos(sol_3_new_exp_up.theta6(num_3_new_exp_up)), -leg32*sin(sol_3_new_exp_up.theta6(num_3_new_exp_up))]'));
%     
%     % if unique, break and stop
%     Unique = judge_unique(0/180*pi, 120/180*pi, 240/180*pi, R_outer_circle_down, R_outer_circle_up, leg11, leg21, leg31, ...
%         sol_1_new_exp_up.theta1(num_1_new_exp_up), sol_2_new_exp_up.theta2(num_2_new_exp_up), sol_3_new_exp_up.theta3(num_3_new_exp_up), double(bbb1 + r11 + r12), double(bbb2 + r21 + r22), double(bbb3 + r31 + r32), ...
%         phi_new_exp_up(1), phi_new_exp_up(2), phi_new_exp_up(3));
%     if Unique
%         disp("UNIQUE CONFIGURATION");
%         break;
%     end
%     
%     % set r in frame {a}
%     r11_a_exp = [r11_a_exp, real(double(R_final_down*[leg11*cos(sol_1_new_exp_up.theta1(num_1_new_exp_up)), 0, -leg11*sin(sol_1_new_exp_up.theta1(num_1_new_exp_up))]'))];
%     r21_a_exp = [r21_a_exp, real(double(R_final_down*[-1/2*leg21*cos(sol_2_new_exp_up.theta2(num_2_new_exp_up)), sqrt(3)/2*leg21*cos(sol_2_new_exp_up.theta2(num_2_new_exp_up)), -leg21*sin(sol_2_new_exp_up.theta2(num_2_new_exp_up))]'))];
%     r31_a_exp = [r31_a_exp, real(double(R_final_down*[-1/2*leg31*cos(sol_3_new_exp_up.theta3(num_3_new_exp_up)), -sqrt(3)/2*leg31*cos(sol_3_new_exp_up.theta3(num_3_new_exp_up)), -leg31*sin(sol_3_new_exp_up.theta3(num_3_new_exp_up))]'))];
% 
%     % draw the upper robot
%     if mod(count, 100) == 0 || count == 1
%         axis equal
%         upper_draw_base_3bar(b_coordinate_source,R_final_down*p, R_final_down*R,c1,c2,c3,c4,c5,c6,'change_color','blue',"coordinate",0);
%         upper_draw_move_3bar(b_coordinate_source, R_final_down*bbb1, R_final_down*bbb2, R_final_down*bbb3, r11_a_exp(:,end), r21_a_exp(:,end), r31_a_exp(:,end), R_final_down*p, R_final_down*R, c11, c22, c33);
%         1
%     end
% 
%     r11_exp = [r11_exp, r11];
%     r21_exp = [r21_exp, r21];
%     r31_exp = [r31_exp, r31];
% 
%     count = count + 1;
% end

load('r11_a_exp.mat');
load('r11_exp.mat');
load('r21_a_exp.mat');
load('r21_exp.mat');
load('r31_a_exp.mat');
load('r31_exp.mat');


% % draw initial configuration
% axis equal
% upper_draw_base_3bar(b_coordinate_source,R_final_down*p_initial_up, R_final_down*R_initial_up,c1,c2,c3,c4,c5,c6,'change_color',color_choose,'coordinate',0,'transparency',0.3);
% upper_draw_move_3bar(b_coordinate_source, R_final_down*bbb1, R_final_down*bbb2, R_final_down*bbb3, r11_a_exp(:, 1), r21_a_exp(:, 1), r31_a_exp(:, 1), R_final_down*p_initial_up, R_final_down*R_initial_up, c11, c22, c33,'change_color',color_choose,'transparency',0.3);

% % get actual R and p using forward kinematics
% count = 1;
% theta4_old_actual_up = sol_1_initial_up.theta4(num_1_initial_up);
% theta5_old_actual_up = sol_2_initial_up.theta5(num_2_initial_up);
% theta6_old_actual_up = sol_3_initial_up.theta6(num_3_initial_up);
% 
% phi_new_exp_x = phi_exp_x_up(count);  % DOF_1
% phi_new_exp_y = phi_exp_y_up(count);  % DOF_2
% p_new_exp_z = p_exp_z_up(count);  % DOF_3
% % *************************************
% [R_new_exp_up, p_new_exp_up, phi_new_exp_up] = upper_inverse_kinematic_configuration(phi_new_exp_x, phi_new_exp_y, p_new_exp_z, R_outer_circle_up);

% triangle1_up_all = p_new_exp_up+R_new_exp_up*c11;
% triangle2_up_all = p_new_exp_up+R_new_exp_up*c22;
% triangle3_up_all = p_new_exp_up+R_new_exp_up*c33;
% r12_actual_all = zeros([3,1]);
% r22_actual_all = zeros([3,1]);
% r32_actual_all = zeros([3,1]);

% count = 2;
% while count <= COUNT
%     
% 
%     % forward kinematic of upper robot
%     syms theta4 theta5 theta6; 
%     
%     r12 = [leg12*cos(theta4), 0, -leg12*sin(theta4)]';
%     r22 = [-1/2*leg22*cos(theta5), sqrt(3)/2*leg22*cos(theta5), -leg22*sin(theta5)]';
%     r32 = [-1/2*leg32*cos(theta6), -sqrt(3)/2*leg32*cos(theta6), -leg32*sin(theta6)]';
%     
%     eqn_1 = eval(norm((bbb1 + r11_exp(:, count) + r12)-(bbb2 + r21_exp(:, count) + r22)))^2 - 3*R_outer_circle_up*R_outer_circle_up == 0;
%     eqn_2 = eval(norm((bbb1 + r11_exp(:, count) + r12)-(bbb3 + r31_exp(:, count) + r32)))^2 - 3*R_outer_circle_up*R_outer_circle_up == 0;
%     eqn_3 = eval(norm((bbb2 + r21_exp(:, count) + r22)-(bbb3 + r31_exp(:, count) + r32)))^2 - 3*R_outer_circle_up*R_outer_circle_up == 0;
%     
%     vars = [theta4, theta5, theta6];
%     % Newton method
%     X_old = [double(theta4_old_actual_up), double(theta5_old_actual_up), double(theta6_old_actual_up)]';
%     F = [eval(norm((bbb1 + r11_exp(:, count) + r12)-(bbb2 + r21_exp(:, count) + r22)))^2 - 3*R_outer_circle_up*R_outer_circle_up;
%     eval(norm((bbb1 + r11_exp(:, count) + r12)-(bbb3 + r31_exp(:, count) + r32)))^2 - 3*R_outer_circle_up*R_outer_circle_up;
%     eval(norm((bbb2 + r21_exp(:, count) + r22)-(bbb3 + r31_exp(:, count) + r32)))^2 - 3*R_outer_circle_up*R_outer_circle_up;];
%     F_diff = jacobian(F, vars);
%     error = 1e-8;
%     X_cur = X_old;
%     while 1
%         F_value = double(subs(F, {theta4 theta5 theta6}, {X_cur(1), X_cur(2), X_cur(3)}));
%         if norm(F_value) <= error
%             break;
%         else
%             F_diff_value = double(subs(F_diff, {theta4 theta5 theta6}, {X_cur(1), X_cur(2), X_cur(3)}));
%             dk = -eye(3,3)/F_diff_value*F_value;
%             X_cur = X_cur + dk
%         end
%     end
%     
%     X_cur
%     r12 = [leg12*cos(X_cur(1)), 0, -leg12*sin(X_cur(1))]';
%     r22 = [-1/2*leg22*cos(X_cur(2)), sqrt(3)/2*leg22*cos(X_cur(2)), -leg22*sin(X_cur(2))]';
%     r32 = [-1/2*leg32*cos(X_cur(3)), -sqrt(3)/2*leg32*cos(X_cur(3)), -leg32*sin(X_cur(3))]';
% 
%     r11_a = r11_a_exp(:, count);
%     r21_a = r21_a_exp(:, count);
%     r31_a = r31_a_exp(:, count);
% 
%     % Centre of the upper circle actually
%     triangle1 = r11_exp(:, count)+r12+bbb1;
%     triangle2 = r21_exp(:, count)+r22+bbb2;
%     triangle3 = r31_exp(:, count)+r32+bbb3;
% 
%     triangle1_up_all = [triangle1_up_all, triangle1];
%     triangle2_up_all = [triangle2_up_all, triangle2];
%     triangle3_up_all = [triangle3_up_all, triangle3];
%     r12_actual_all = [r12_actual_all, r12];
%     r22_actual_all = [r22_actual_all, r22];
%     r32_actual_all = [r32_actual_all, r32];
% 
% 
%     % draw the upper robot
% 
%     if mod(count, 50) == 0
%         phi_new_exp_x = phi_exp_x_up(count);  % DOF_1
%         phi_new_exp_y = phi_exp_y_up(count);  % DOF_2
%         p_new_exp_z = p_exp_z_up(count);  % DOF_3
%         % *************************************
%         [R_new_exp_up, p_new_exp_up, phi_new_exp_up] = upper_inverse_kinematic_configuration(phi_new_exp_x, phi_new_exp_y, p_new_exp_z, R_outer_circle_up);
%     
%         % get initital r and draw the initial configuration of 3D
%         % calculate the second rotation joint on the middle in frame {b}
%         p = p_new_exp_up;
%         R = R_new_exp_up;
% 
%         traceA = R_final_down * (p+R*c11) + b_coordinate_source';
%         traceB = R_final_down * (p+R*c22) + b_coordinate_source';
%         traceC = R_final_down * (p+R*c33) + b_coordinate_source';
%         plot3(traceA(1),traceA(2),traceA(3),    ...
%         'Marker','o', ...
%         'MarkerSize',2,...
%         'MarkerEdgeColor',dot_color,...
%         'MarkerFaceColor',dot_color);
%         plot3(traceB(1),traceB(2),traceB(3), ...
%         'Marker','o', ...
%         'MarkerSize',2,...
%         'MarkerEdgeColor',dot_color,...
%         'MarkerFaceColor',dot_color);
%         plot3(traceC(1),traceC(2),traceC(3), ...
%         'Marker','o', ...
%         'MarkerSize',2,...
%         'MarkerEdgeColor',dot_color,...
%         'MarkerFaceColor',dot_color);
%     end
% 
% %     if mod(count, 50) == 0
% %             traceA = R_final_down * (r11_exp(:, count)+r12+bbb1) + b_coordinate_source';
% %             traceB = R_final_down * (r21_exp(:, count)+r22+bbb2) + b_coordinate_source';
% %             traceC = R_final_down * (r31_exp(:, count)+r32+bbb3) + b_coordinate_source';
% %             plot3(traceA(1),traceA(2),traceA(3),    ...
% %             'Marker','o', ...
% %             'MarkerSize',1,...
% %             'MarkerEdgeColor','b',...
% %             'MarkerFaceColor','b');
% %             plot3(traceB(1),traceB(2),traceB(3), ...
% %             'Marker','o', ...
% %             'MarkerSize',1,...
% %             'MarkerEdgeColor','b',...
% %             'MarkerFaceColor','b');
% %             plot3(traceC(1),traceC(2),traceC(3), ...
% %             'Marker','o', ...
% %             'MarkerSize',1,...
% %             'MarkerEdgeColor','b',...
% %             'MarkerFaceColor','b');
% %             disp(1);
% %     end
% 
%     if count == COUNT || count == 2
% 
% %         phi_new_exp_x = phi_exp_x_up(count);  % DOF_1
% %         phi_new_exp_y = phi_exp_y_up(count);  % DOF_2
% %         p_new_exp_z = p_exp_z_up(count);  % DOF_3
% %         % *************************************
% %         [R_new_exp_up, p_new_exp_up, phi_new_exp_up] = upper_inverse_kinematic_configuration(phi_new_exp_x, phi_new_exp_y, p_new_exp_z, R_outer_circle_up);
% %     
% %         % get initital r and draw the initial configuration of 3D
% %         % calculate the second rotation joint on the middle in frame {b}
% %         p = p_new_exp_up;
% %         R = R_new_exp_up;
% % 
% %         % inverse kinematics
% %         upper_draw_base_3bar(b_coordinate_source,R_final_down*p, R_final_down*R,c1,c2,c3,c4,c5,c6,'change_color',color_choose,"coordinate",0);          
% 
%         if count == 2
%             transparency = 0.3;
%         else
%             transparency = 1;
%         end
% 
%         triangle1_a = R_final_down * (r11_exp(:, count)+r12+bbb1) + b_coordinate_source';
%         triangle2_a = R_final_down * (r21_exp(:, count)+r22+bbb2) + b_coordinate_source';
%         triangle3_a = R_final_down * (r31_exp(:, count)+r32+bbb3) + b_coordinate_source';
% 
%         upper_draw_line(triangle1_a,triangle2_a,'change_color',color_choose,'transparency',transparency);
%         upper_draw_line(triangle2_a,triangle3_a,'change_color',color_choose,'transparency',transparency);
%         upper_draw_line(triangle1_a,triangle3_a,'change_color',color_choose,'transparency',transparency);
% 
%         leg11_draw1 = R_final_down * bbb1 + b_coordinate_source';
%         leg21_draw1 = R_final_down * bbb2 + b_coordinate_source';
%         leg31_draw1 = R_final_down * bbb3 + b_coordinate_source';
% 
%         leg11_draw2 = R_final_down * (r11_exp(:, count)+bbb1) + b_coordinate_source';
%         leg21_draw2 = R_final_down * (r21_exp(:, count)+bbb2) + b_coordinate_source';
%         leg31_draw2 = R_final_down * (r31_exp(:, count)+bbb3) + b_coordinate_source';
% 
%         leg12_draw2 = R_final_down * (r11_exp(:, count)+bbb1+r12) + b_coordinate_source';
%         leg22_draw2 = R_final_down * (r21_exp(:, count)+bbb2+r22) + b_coordinate_source';
%         leg32_draw2 = R_final_down * (r31_exp(:, count)+bbb3+r32) + b_coordinate_source';
% 
%         upper_draw_line(leg11_draw1, leg11_draw2,'change_color',color_choose,'transparency',transparency);
%         upper_draw_line(leg21_draw1, leg21_draw2,'change_color',color_choose,'transparency',transparency);
%         upper_draw_line(leg31_draw1, leg31_draw2,'change_color',color_choose,'transparency',transparency);
% 
%         upper_draw_line(leg11_draw2, leg12_draw2,'change_color',color_choose,'transparency',transparency);
%         upper_draw_line(leg21_draw2, leg22_draw2,'change_color',color_choose,'transparency',transparency);
%         upper_draw_line(leg31_draw2, leg32_draw2,'change_color',color_choose,'transparency',transparency);
%         disp("plot");
%     end
%     
%     theta4_old_actual_up = X_cur(1);
%     theta5_old_actual_up = X_cur(2);
%     theta6_old_actual_up = X_cur(3);
%     count = count + 1;
% end



load('r12_actual_all.mat')
load('r22_actual_all.mat')
load('r32_actual_all.mat')
load('triangle1_up_all.mat');
load('triangle2_up_all.mat');
load('triangle3_up_all.mat');

count = 2;
while count <= COUNT
    % draw the upper robot

    if mod(count, 50) == 0
        phi_new_exp_x = phi_exp_x_up(count);  % DOF_1
        phi_new_exp_y = phi_exp_y_up(count);  % DOF_2
        p_new_exp_z = p_exp_z_up(count);  % DOF_3
        % *************************************
        [R_new_exp_up, p_new_exp_up, phi_new_exp_up] = upper_inverse_kinematic_configuration(phi_new_exp_x, phi_new_exp_y, p_new_exp_z, R_outer_circle_up);
    
        % get initital r and draw the initial configuration of 3D
        % calculate the second rotation joint on the middle in frame {b}
        p = p_new_exp_up;
        R = R_new_exp_up;

        traceA = R_final_down * (p+R*c11) + b_coordinate_source';
        traceB = R_final_down * (p+R*c22) + b_coordinate_source';
        traceC = R_final_down * (p+R*c33) + b_coordinate_source';
        plot3(traceA(1),traceA(2),traceA(3),    ...
        'Marker','o', ...
        'MarkerSize',2,...
        'MarkerEdgeColor',dot_color,...
        'MarkerFaceColor',dot_color);
        plot3(traceB(1),traceB(2),traceB(3), ...
        'Marker','o', ...
        'MarkerSize',2,...
        'MarkerEdgeColor',dot_color,...
        'MarkerFaceColor',dot_color);
        plot3(traceC(1),traceC(2),traceC(3), ...
        'Marker','o', ...
        'MarkerSize',2,...
        'MarkerEdgeColor',dot_color,...
        'MarkerFaceColor',dot_color);
    end



    if count == COUNT || count == 2
        if count == 2
            transparency = 0.3;
        else
            transparency = 1;
        end

        triangle1_a = R_final_down * (r11_exp(:, count)+r12_actual_all(:, count)+bbb1) + b_coordinate_source';
        triangle2_a = R_final_down * (r21_exp(:, count)+r22_actual_all(:, count)+bbb2) + b_coordinate_source';
        triangle3_a = R_final_down * (r31_exp(:, count)+r32_actual_all(:, count)+bbb3) + b_coordinate_source';

        upper_draw_line(triangle1_a,triangle2_a,'change_color',color_choose,'transparency',transparency);
        upper_draw_line(triangle2_a,triangle3_a,'change_color',color_choose,'transparency',transparency);
        upper_draw_line(triangle1_a,triangle3_a,'change_color',color_choose,'transparency',transparency);

        leg11_draw1 = R_final_down * bbb1 + b_coordinate_source';
        leg21_draw1 = R_final_down * bbb2 + b_coordinate_source';
        leg31_draw1 = R_final_down * bbb3 + b_coordinate_source';

        leg11_draw2 = R_final_down * (r11_exp(:, count)+bbb1) + b_coordinate_source';
        leg21_draw2 = R_final_down * (r21_exp(:, count)+bbb2) + b_coordinate_source';
        leg31_draw2 = R_final_down * (r31_exp(:, count)+bbb3) + b_coordinate_source';

        leg12_draw2 = R_final_down * (r11_exp(:, count)+bbb1+r12_actual_all(:, count)) + b_coordinate_source';
        leg22_draw2 = R_final_down * (r21_exp(:, count)+bbb2+r22_actual_all(:, count)) + b_coordinate_source';
        leg32_draw2 = R_final_down * (r31_exp(:, count)+bbb3+r32_actual_all(:, count)) + b_coordinate_source';

        upper_draw_line(leg11_draw1, leg11_draw2,'change_color',color_choose,'transparency',transparency);
        upper_draw_line(leg21_draw1, leg21_draw2,'change_color',color_choose,'transparency',transparency);
        upper_draw_line(leg31_draw1, leg31_draw2,'change_color',color_choose,'transparency',transparency);

        upper_draw_line(leg11_draw2, leg12_draw2,'change_color',color_choose,'transparency',transparency);
        upper_draw_line(leg21_draw2, leg22_draw2,'change_color',color_choose,'transparency',transparency);
        upper_draw_line(leg31_draw2, leg32_draw2,'change_color',color_choose,'transparency',transparency);
        disp("plot");
    end

    count = count + 1;
end


[center_p, actual_R, phix, phiy, phiz] = get_actual_configuration(triangle1_up_all(:,end), triangle2_up_all(:,end), triangle3_up_all(:,end), R_outer_circle_up, c11, c22, c33);
hold on;
upper_draw_line(b_coordinate_source'+R_final_down*center_p, b_coordinate_source'+R_final_down*(center_p + actual_R*[0, 0, -150]'), 'change_color', 'k');



% figure(2)
% hold on
% set(gca,'FontSize',20,'FontName','Arial');
% xlim([0,55]);
% ylim([0,450]);
% xlabel("Time(seconds)",FontSize=20);
% ylabel("Displacement",FontSize=20);
% T = 0:0.01:55;
% plot(T,[px_expect, 55*ones([1, 500])],...
%     'LineWidth',2);
% plot(T,[py_expect, 430*ones([1,500])],...
%     'LineWidth',2);
% plot(T,[phi_expect, 40*ones([1, 500])],...
%     'LineStyle','--', ...
%     'LineWidth',2);
% legend(["p_x(mm)", "p_y(mm)","\psi_z(rad)"], 'FontSize',15);
% hold off

% figure(3)
% hold on
% diff_p = p_actual-[px_expect; py_expect];
% diff_px = diff_p(1,:);
% diff_py = diff_p(2,:);
% set(gca,'FontSize',20,'FontName','Arial');
% xlim([-1,58]);
% ylim([-0.1,0.8]);
% xlabel("Time(seconds)",FontSize=20);
% ylabel("Error",FontSize=20);
% T = 0:0.01:55;
% plot(T,[diff_px, diff_px(end)*ones([1, 500])],...
%     'LineWidth',2);
% plot(T,[diff_py, diff_py(end)*ones([1, 500])],...
%     'LineWidth',2);
% legend(["\Deltax(mm)", "\Deltay(mm)"], 'FontSize',15);

% figure(1)
% T = 0:0.01:50;
% plotx{1} = T;
% ploty{1} = p_actual(1, :);
% plotx{2} = T;
% ploty{2} = px_expect;
% opt.XLabel{1} = 'Time(s)';
% opt.YLabel{1} = 'Displacement(mm)';
% opt.Colors = [40/255, 120/255, 181/255; 30/255, 30/255, 32/255;];
% opt.LineWidth = [2, 2];
% opt.LineStyle = {'-', '--'};
% opt.Legend = {'Simulation', 'Desire'};
% plotPub(plotx, ploty, 2, opt);

% figure(2)
% T = 0:0.01:50;
% yyaxis left
% plot(T, p_actual(1, :),'color','#8983BF','LineStyle','-','LineWidth',2);
% hold on;
% plot(T, px_expect,'color','#FFBE7A','LineStyle','--','LineWidth',2);
% plot(T, 0.1*p_actual(2, :), 'color','#9DC3E7','LineStyle','-','LineWidth',2);
% 
% plot(T, 0.1*py_expect,'color','#EF7A6D','LineStyle','--','LineWidth',2);
% 
% xlabel('Time(s)');
% ylabel('Distance(mm)');
% set(gca,'ycolor','k');
% set(gca,'FontSize',15,'FontName','Arial');
% 
% hold on
% yyaxis right
% plot(T, phi_actual,'LineStyle','-','LineWidth',2,'Color','#32B897');
% plot(T, phi_expect,'color','#999999','LineStyle','--','LineWidth',2);
% legend(["Actual p_x(mm)", "Target p_x(mm)", "Actual p_y(mm)*0.1","Target p_y(mm)*0.1", "Actual phi(degree)", "Target phi(degree)"], 'FontSize',15);
% ylabel('Angle(degree)');
% xlim([0,50]);
% ylim([0, 60]);
% set(gca,'ycolor','k');

% plotx{1} = T;
% ploty{1} = p_actual(1, :);
% plotx{2} = T;
% ploty{2} = px_expect;
% opt.XLabel{1} = 'Time(s)';
% opt.YLabel{1} = 'Displacement(mm)';
% opt.Colors = [40/255, 120/255, 181/255; 30/255, 30/255, 32/255;];
% opt.LineWidth = [2, 2];
% opt.LineStyle = {'-', '--'};
% opt.Legend = {'Simulation', 'Desire'};
% plotPub(plotx, ploty, 2, opt);



% 
% figure(2)
% T = 0:0.01:50;
% plotx{1} = T;
% ploty{1} = p_actual(2, :);
% plotx{2} = T;
% ploty{2} = py_expect;
% opt.XLabel = 'Time(s)';
% opt.YLabel = 'p_y(mm)';
% opt.Colors = [40/255, 120/255, 181/255; 30/255, 30/255, 32/255;];
% opt.LineWidth = [2, 2];
% opt.LineStyle = {'-', '--'};
% opt.Legend = {'Simulation', 'Desire'};
% plotPub(plotx, ploty, 2, opt);
% 
% figure(3)
% T = 0:0.01:50;
% plotx{1} = T;
% ploty{1} = phi_actual;
% plotx{2} = T;
% ploty{2} = phi_expect;
% opt.XLabel = 'Time(s)';
% opt.YLabel = 'phi(degree)';
% opt.Colors = [40/255, 120/255, 181/255; 30/255, 30/255, 32/255;];
% opt.LineWidth = [2, 2];
% opt.LineStyle = {'-', '--'};
% opt.Legend = {'Simulation', 'Desire'};
% plotPub(plotx, ploty, 2, opt);


 
% diff_p = abs(p_actual-[px_expect; py_expect]);
% diff_px = diff_p(1,:);
% diff_py = diff_p(2,:);
% diff_phi = abs(phi_expect-phi_actual);
% error_diff_px = sum(diff_px)/length(diff_px);
% error_diff_py = sum(diff_py)/length(diff_py);
% error_diff_phi = sum(diff_phi)/length(diff_phi);



% figure(1)
T = 0:0.4:500;
T_print = 0:0.04:50;
s = 0.005*T - 0.1*sin(0.2*pi*T)/pi;
phi_exp_x_up = -4/45*pi*s;
phi_exp_x_up = phi_exp_x_up/pi*180;
phi_exp_y_up = pi/15*s;
phi_exp_y_up = phi_exp_y_up/pi*180;
p_exp_z_up = -28*s - 150;
% t = 0:0.4:500;
% s = 0.005*t - 0.1*sin(0.2*pi*t)/pi;
% phi_exp_x_up = -4/45*pi*s;
% phi_exp_y_up = pi/15*s;
% p_exp_z_up = -28*s - 150;


phi_actual_x_up = [];
phi_actual_y_up = [];
p_actual_up = [];
% get actual configuration
% count = 1;
% while count <= COUNT
%     [center_p_, actual_R_, phix, phiy, phiz] = get_actual_configuration(triangle1_up_all(:,count), triangle2_up_all(:,count), triangle3_up_all(:,count), R_outer_circle_up,c11,c22,c33);
%     phi_actual_x_up = [phi_actual_x_up, phix/pi*180];
%     phi_actual_y_up = [phi_actual_y_up, phiy/pi*180];
%     p_actual_up = [p_actual_up,center_p];
%     count = count + 1; 
% end

% load('phi_actual_x_up.mat');
% load('phi_actual_y_up.mat');
% 
% plotx{1} = T_print;
% ploty{1} = phi_actual_x_up;
% plotx{2} = T_print;
% ploty{2} = phi_exp_x_up;
% opt.XLabel = 'Time(s)';
% opt.YLabel = 'Angle(degree)';
% opt.Colors = [40/255, 120/255, 181/255; 30/255, 30/255, 32/255;];
% opt.LineWidth = [2, 2];
% opt.LineStyle = {'-', '--'};
% opt.Legend = {'Actual \phi_x', 'Target \phi_x'};
% plotPub(plotx, ploty, 2, opt);
% 
% 
% plotx{1} = T_print;
% ploty{1} = phi_actual_y_up;
% plotx{2} = T_print;
% ploty{2} = phi_exp_y_up;
% opt.XLabel = 'Time(s)';
% opt.YLabel = 'Angle(degree)';
% opt.Colors = [40/255, 120/255, 181/255; 30/255, 30/255, 32/255;];
% opt.LineWidth = [2, 2];
% opt.LineStyle = {'-', '--'};
% opt.Legend = {'Actual \phi_y', 'Target \phi_y'};
% plotPub(plotx, ploty, 2, opt);
% 
% 
% diff_phi_x_up = abs(phi_actual_x_up-phi_exp_x_up);
% diff_phi_y_up = abs(phi_actual_y_up-phi_exp_y_up);
% error_phi_x_up = sum(diff_phi_x_up)/length(diff_phi_x_up)
% error_phi_y_up = sum(diff_phi_y_up)/length(diff_phi_y_up)

% plotx{1} = T_print;
% ploty{1} = phi_exp_x_up;
% plotx{2} = T_print;
% ploty{2} = phi_exp_x_up;
% opt.XLabel = 'Time(s)';
% opt.YLabel = 'phi_x(mm)';
% opt.Colors = [40/255, 120/255, 181/255; 30/255, 30/255, 32/255;];
% opt.LineWidth = [2, 2];
% opt.LineStyle = {'-', '--'};
% opt.Legend = {'Simulation', 'Desired'};
% plotPub(plotx, ploty, 2, opt);


% load('p_actual_up.mat');
% plotx{1} = T_print;
% ploty{1} = p_actual_up(3, :);
% plotx{2} = T_print;
% ploty{2} = p_exp_z_up;
% opt.XLabel = 'Time(s)';
% opt.YLabel = 'p_z(mm)';
% opt.Colors = [40/255, 120/255, 181/255; 30/255, 30/255, 32/255;];
% opt.LineWidth = [2, 2];
% opt.LineStyle = {'-', '--'};
% opt.Legend = {'Simulation', 'Desire'};
% plotPub(plotx, ploty, 2, opt);


% diff_p_up = abs(p_actual_up(3, :)-p_exp_z_up)
% diff_p_up = sum(diff_p_up)/length(diff_p_up)

% actual_R =[-0.8497    0.1673    0.5000
%     0.1673   -0.8137    0.5567
%     0.5000    0.5567    0.6634];

% phi_target_x_down = -40/180*pi;  % DOF_1
% phi_target_y_down = 30/180*pi;  % DOF_2
% p_target_z_down = -220;  % DOF_3
% % *************************************
% [R_target_up, p_target_up, phi_target_up] = upper_inverse_kinematic_configuration(phi_target_x_down, phi_target_y_down, p_target_z_down, R_outer_circle_up);

% 
% % 
% draw_coordinate_system(100,R_final_down*actual_R,[a_coordinate_source(1),a_coordinate_source(2),0]','rgb',"A");
% draw_coordinate_system(100,R_final_down*R_target_up,[a_coordinate_source(1)+100,a_coordinate_source(2),0]','rgb',"E");


