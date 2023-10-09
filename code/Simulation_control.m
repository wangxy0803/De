%%
% Simulation
clc;
clear;
close all;

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
    draw_base_3bar(a_coordinate_source,'{b}',p,R,b1,b2,b3,b4,b5,b6,len,width,rectangular_W,phi,'change_color','b');
    draw_move_3bar(d_1x_down,d_2x_down,d_3x_down,d_1y_initial_down,d_2y_initial_down,d_3y_initial_down,width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R,p,'change_color','b');
else
    error('slider1 is out of the limitation!');
end


% draw skull
hold on
TR = stlread('skull.stl');
P = TR.Points;% vertices information
P = P*+30;
P(:,1)=P(:,1)+60;
P(:,2)=P(:,2)+230;
P(:,3)=P(:,3)-380;
CL = TR.ConnectivityList;% connection list
patch('vertices', P, 'faces', CL, 'facevertexcdata',P(:,3), 'facecolor', 'interp', 'edgecolor', 'none');%以z方向坐标作为颜色画出立体图
colormap("bone");view(-30,40);axis('equal');

light('Position',[-160 1000 100000],'Style','local');
light('Position',[-160 0 100000],'Style','local');
light('Position',[-160 -1000 100000],'Style','local');
light('Position',[160 1000 100000],'Style','local');
light('Position',[160 0 100000],'Style','local');
light('Position',[160 -1000 100000],'Style','local');


% 
% % ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————
% % upper robot
% hold on
% 
% % set parameters
% % the first rotation joint discribed in frame {b}
% R_outer_circle_down = r_hexagon*0.8;
% bbb1 = [R_outer_circle_down*cos(0/180*pi), R_outer_circle_down*sin(0/180*pi), 0]';
% bbb2 = [R_outer_circle_down*cos(120/180*pi), R_outer_circle_down*sin(120/180*pi), 0]';
% bbb3 = [R_outer_circle_down*cos(240/180*pi), R_outer_circle_down*sin(240/180*pi), 0]';
% % configuration of the upper platform in frame {c}
% c1 = [r_hexagon*1.2*cos(60/180*pi), r_hexagon*1.2*sin(60/180*pi), 0];
% c2 = [r_hexagon*1.2*cos(120/180*pi), r_hexagon*1.2*sin(120/180*pi), 0];
% c3 = [r_hexagon*1.2*cos(180/180*pi), r_hexagon*1.2*sin(180/180*pi), 0];
% c4 = [r_hexagon*1.2*cos(240/180*pi), r_hexagon*1.2*sin(240/180*pi), 0];
% c5 = [r_hexagon*1.2*cos(300/180*pi), r_hexagon*1.2*sin(300/180*pi), 0];
% c6 = [r_hexagon*1.2*cos(360/180*pi), r_hexagon*1.2*sin(360/180*pi), 0];
% % the second rotation joint on the upper platform in frame {c}
% R_outer_circle_up = 60;
% c11 = [R_outer_circle_up*cos(0/180*pi), R_outer_circle_up*sin(0/180*pi), 0]';
% c22 = [R_outer_circle_up*cos(120/180*pi), R_outer_circle_up*sin(120/180*pi), 0]';
% c33 = [R_outer_circle_up*cos(240/180*pi), R_outer_circle_up*sin(240/180*pi), 0]';
% b_coordinate_source = [a_coordinate_source(1)+p(1),a_coordinate_source(2)+p(2),0];
% % set length of legs
% leg11 = 200;
% leg12 = 200;
% leg21 = 200;
% leg22 = 200;
% leg31 = 200;
% leg32 = 200;
% 
% 
% 
% % set initial configuration in frame {b}
% % *************************************
% phi_initial_up_x = 0/180*pi;  % DOF_1
% phi_initial_up_y = 0/180*pi;  % DOF_2
% p_initial_up_z = -150;  % DOF_3
% % *************************************
% [R_initial_up, p_initial_up, ~] = upper_inverse_kinematic_configuration(phi_initial_up_x, phi_initial_up_y, p_initial_up_z, R_outer_circle_up);
% 
% % get initital r and draw the initial configuration of 3D
% % calculate the second rotation joint on the middle in frame {b}
% p = p_initial_up;
% R = R_initial_up;
% [sol_1_initial, sol_2_initial, sol_3_initial, num_1_initial, num_2_initial, num_3_initial] = upper_inverse_kinematic_solve_equations(...
%     p, R, R_outer_circle_down, R_outer_circle_up, leg11, leg12, leg21, leg22, leg31, leg32);
% 
%  % r in frame {b}
% r11 = real(double([leg11*cos(sol_1_initial.theta1(num_1_initial)), 0, -leg11*sin(sol_1_initial.theta1(num_1_initial))]'));
% r12 = real(double([leg12*cos(sol_1_initial.theta4(num_1_initial)), 0, -leg12*sin(sol_1_initial.theta4(num_1_initial))]'));
% r21 = real(double([-1/2*leg21*cos(sol_2_initial.theta2(num_2_initial)), sqrt(3)/2*leg21*cos(sol_2_initial.theta2(num_2_initial)), -leg21*sin(sol_2_initial.theta2(num_2_initial))]'));
% r22 = real(double([-1/2*leg22*cos(sol_2_initial.theta5(num_2_initial)), sqrt(3)/2*leg22*cos(sol_2_initial.theta5(num_2_initial)), -leg22*sin(sol_2_initial.theta5(num_2_initial))]'));
% r31 = real(double([-1/2*leg31*cos(sol_3_initial.theta3(num_3_initial)), -sqrt(3)/2*leg31*cos(sol_3_initial.theta3(num_3_initial)), -leg31*sin(sol_3_initial.theta3(num_3_initial))]'));
% r32 = real(double([-1/2*leg32*cos(sol_3_initial.theta6(num_3_initial)), -sqrt(3)/2*leg32*cos(sol_3_initial.theta6(num_3_initial)), -leg32*sin(sol_3_initial.theta6(num_3_initial))]'));
% 
% 
% % p_cal = [0.75363138351776710218651211860664; 0.066607552994114670347130943216457; 0];
% R_initial_up_a = [R_initial_down(1,1), R_initial_down(1,2), 0;
%         R_initial_down(2,1),  R_initial_down(2,2), 0;
%         0, 0, 1] * R_initial_up;
% 
% % set r in frame {a}
% r11_a = real(double(R_initial_up_a*[leg11*cos(sol_1_initial.theta1(num_1_initial)), 0, -leg11*sin(sol_1_initial.theta1(num_1_initial))]'));
% r21_a = real(double(R_initial_up_a*[-1/2*leg21*cos(sol_2_initial.theta2(num_2_initial)), sqrt(3)/2*leg21*cos(sol_2_initial.theta2(num_2_initial)), -leg21*sin(sol_2_initial.theta2(num_2_initial))]'));
% r31_a = real(double(R_initial_up_a*[-1/2*leg31*cos(sol_3_initial.theta3(num_3_initial)), -sqrt(3)/2*leg31*cos(sol_3_initial.theta3(num_3_initial)), -leg31*sin(sol_3_initial.theta3(num_3_initial))]'));
% 
% 
% % draw the upper robot
% axis equal
% upper_draw_base_3bar(b_coordinate_source,'{b}',R_initial_up_a*p, R_initial_up_a*R,c1,c2,c3,c4,c5,c6,'change_color','blue');
% upper_draw_move_3bar(b_coordinate_source, R_initial_up_a*bbb1, R_initial_up_a*bbb2, R_initial_up_a*bbb3, r11_a, r21_a, r31_a, R_initial_up_a*p, R_initial_up_a*R, c11, c22, c33);
% 
%     
% % ——————————————————————————————————————————————————————————————————————————————————————————————————————————————————




% action
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

    
    [d_1y_down,d_2y_down,d_3y_down] = select_d(d_1y_1_down, d_1y_2_down, d_2y_1_down, d_2y_2_down, d_3y_1_down, d_3y_2_down, INVALID_VALUE, a2, a3, rectangular_L, 'configuration_1', 'up');
%     if  i == 2500 || i == length(Tphi)
%         draw_base_3bar(a_coordinate_source,'{b}',p,R,b1,b2,b3,b4,b5,b6,len,width,rectangular_W,phi,'change_color','r', "coordinate", 0);
%         draw_move_3bar(d_1x_down,d_2x_down,d_3x_down,d_1y_down,d_2y_down,d_3y_down,width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R,p,'change_color','r');
%     end
    [TRUE_OR_FALSE, ~] = check_limitation(distance,R*b22'+p, a1(2),d_1y_down,leg_1,width);
    if TRUE_OR_FALSE
        d_1y_expect = [d_1y_expect, d_1y_down]; 
        d_1x_expect = [d_1x_expect, d_1x_down]; 
        d_2y_expect = [d_2y_expect, d_2y_down]; 
        d_2x_expect = [d_2x_expect, d_2x_down];
        d_3y_expect = [d_3y_expect, d_3y_down]; 
        d_3x_expect = [d_3x_expect, d_3x_down];
        if mod(i, 100) == 0
            traceA = p+R*b11';
            traceB = p+R*b22';
            plot(traceA(1),traceA(2),    ...
            'Marker','o', ...
            'MarkerSize',1,...
            'MarkerEdgeColor','r',...
            'MarkerFaceColor',"r");
            plot(traceB(1),traceB(2), ...
            'Marker','o', ...
            'MarkerSize',1,...
            'MarkerEdgeColor','r',...
            'MarkerFaceColor',"r");
        end

        if mod(i, 100) == 0
                traceA = p_actual(:,i)+R_actual(:,(2*i-1):(2*i))*b11';
                traceB = p_actual(:,i)+R_actual(:,(2*i-1):(2*i))*b22';
                plot(traceA(1),traceA(2),    ...
                'Marker','o', ...
                'MarkerSize',1,...
                'MarkerEdgeColor','b',...
                'MarkerFaceColor','b');
                plot(traceB(1),traceB(2), ...
                'Marker','o', ...
                'MarkerSize',1,...
                'MarkerEdgeColor','b',...
                'MarkerFaceColor','b');
        end

        if  i == 2500
            draw_base_3bar(a_coordinate_source,'{b}',p_actual(:,i),R_actual(:,(2*i-1):(2*i)),b1,b2,b3,b4,b5,b6,len,width,rectangular_W,phi_actual(i),'change_color','b', "coordinate", 1);
            draw_move_3bar(d_1x_down,d_2x_down,d_3x_down,d_1y_down,d_2y_down,d_3y_down,width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R_actual(:,(2*i-1):(2*i)),p_actual(:,i),'change_color','b');
        end

        if  i == length(Tphi)
            draw_base_3bar(a_coordinate_source,'{b}',p_actual(:,i),R_actual(:,(2*i-1):(2*i)),b1,b2,b3,b4,b5,b6,len,width,rectangular_W,phi_actual(i),'change_color','b', "coordinate", 1);
            draw_move_3bar(d_1x_down,d_2x_down,d_3x_down,d_1y_down,d_2y_down,d_3y_down,width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R_actual(:,(2*i-1):(2*i)),p_actual(:,i),'change_color','b');
        end

    else
        error('slider1 is out of the limitation!');
    end
end



% --------------------------------------------------------------------------------------------------------------------------------------------------------
% upper action
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
% s = 0.005*t - 0.1*sin(0.2*pi*t)/pi;
% phi_exp_x_up = -1/45*pi*s;
% phi_exp_y_up = pi/15*s;
% p_exp_z_up = -56*s - 150;


% maximum iterations
COUNT = size(t);
COUNT = COUNT(2);
% current iterations
count = 1;

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
% i = 1;
% while i <= COUNT
%      % *************************************
%     phi_new_exp_x = phi_exp_x_up(i);  % DOF_1
%     phi_new_exp_y = phi_exp_y_up(i);  % DOF_2
%     p_new_exp_z = p_exp_z_up(i);  % DOF_3
%     % *************************************
%     [R_new_exp_up, p_new_exp_up, phi_new_exp_up] = upper_inverse_kinematic_configuration(phi_new_exp_x, phi_new_exp_y, p_new_exp_z, R_outer_circle_up);
% 
% 
% 
%     p = p_new_exp_up;
%     R = R_new_exp_up;
%     % draw the upper robot
%     if mod(i, 100) == 0 || i == 1
%         axis equal
%         upper_draw_base_3bar(b_coordinate_source,R_final_down*p, R_final_down*R,c1,c2,c3,c4,c5,c6,'change_color','blue',"coordinate",0);
%         upper_draw_move_3bar(b_coordinate_source, R_final_down*bbb1, R_final_down*bbb2, R_final_down*bbb3, r11_a_exp(:,i), r21_a_exp(:,i), r31_a_exp(:,i), R_final_down*p, R_final_down*R, c11, c22, c33);
%         1
%     end
% 
%     i = i + 1;
% end

axis equal
upper_draw_base_3bar(b_coordinate_source,R_final_down*p_initial_up, R_final_down*R_initial_up,c1,c2,c3,c4,c5,c6,'change_color','blue','coordinate',0);
upper_draw_move_3bar(b_coordinate_source, R_final_down*bbb1, R_final_down*bbb2, R_final_down*bbb3, r11_a_exp(:, 1), r21_a_exp(:, 1), r31_a_exp(:, 1), R_final_down*p_initial_up, R_final_down*R_initial_up, c11, c22, c33);
% get actual R and p using forward kinematics
count = 2;
theta4_old_actual_up = sol_1_initial_up.theta4(num_1_initial_up);
theta5_old_actual_up = sol_2_initial_up.theta5(num_2_initial_up);
theta6_old_actual_up = sol_3_initial_up.theta6(num_3_initial_up);

R_new_actual_up_all = [];
p_new_actual_up_all = [];


while count <= COUNT
    
    % forward kinematic of upper robot
    syms theta4 theta5 theta6; 
    
    r12 = [leg12*cos(theta4), 0, -leg12*sin(theta4)]';
    r22 = [-1/2*leg22*cos(theta5), sqrt(3)/2*leg22*cos(theta5), -leg22*sin(theta5)]';
    r32 = [-1/2*leg32*cos(theta6), -sqrt(3)/2*leg32*cos(theta6), -leg32*sin(theta6)]';
    
    eqn_1 = eval(norm((bbb1 + r11_exp(:, count) + r12)-(bbb2 + r21_exp(:, count) + r22)))^2 - 3*R_outer_circle_up*R_outer_circle_up == 0;
    eqn_2 = eval(norm((bbb1 + r11_exp(:, count) + r12)-(bbb3 + r31_exp(:, count) + r32)))^2 - 3*R_outer_circle_up*R_outer_circle_up == 0;
    eqn_3 = eval(norm((bbb2 + r21_exp(:, count) + r22)-(bbb3 + r31_exp(:, count) + r32)))^2 - 3*R_outer_circle_up*R_outer_circle_up == 0;
    
    vars = [theta4, theta5, theta6];
    % Newton method
    X_old = [double(theta4_old_actual_up), double(theta5_old_actual_up), double(theta6_old_actual_up)]';
    F = [eval(norm((bbb1 + r11_exp(:, count) + r12)-(bbb2 + r21_exp(:, count) + r22)))^2 - 3*R_outer_circle_up*R_outer_circle_up;
    eval(norm((bbb1 + r11_exp(:, count) + r12)-(bbb3 + r31_exp(:, count) + r32)))^2 - 3*R_outer_circle_up*R_outer_circle_up;
    eval(norm((bbb2 + r21_exp(:, count) + r22)-(bbb3 + r31_exp(:, count) + r32)))^2 - 3*R_outer_circle_up*R_outer_circle_up;];
    F_diff = jacobian(F, vars);
    error = 1e-8;
    X_cur = X_old;
    while 1
        F_value = double(subs(F, {theta4 theta5 theta6}, {X_cur(1), X_cur(2), X_cur(3)}));
        if norm(F_value) <= error
            break;
        else
            F_diff_value = double(subs(F_diff, {theta4 theta5 theta6}, {X_cur(1), X_cur(2), X_cur(3)}));
            dk = -eye(3,3)/F_diff_value*F_value;
            X_cur = X_cur + dk
        end
    end
    
    X_cur
    r12 = [leg12*cos(X_cur(1)), 0, -leg12*sin(X_cur(1))]';
    r22 = [-1/2*leg22*cos(X_cur(2)), sqrt(3)/2*leg22*cos(X_cur(2)), -leg22*sin(X_cur(2))]';
    r32 = [-1/2*leg32*cos(X_cur(3)), -sqrt(3)/2*leg32*cos(X_cur(3)), -leg32*sin(X_cur(3))]';

%     syms phi_actual_x phi_actual_y p_actual_z
%     [R_new_actual_up, p_new_actual_up, phi_new_actual_up] = upper_inverse_kinematic_configuration(phi_actual_x, phi_actual_y, p_actual_z, R_outer_circle_up);
%     R = R_new_actual_up;
%     p = p_new_actual_up;
% 
% %     eqn1 = p+R*c11 == bbb1+r11_exp(:, count)+r12;
% %     eqn2 = p+R*c22 == bbb2+r21_exp(:, count)+r22;
% %     eqn3 = p+R*c33 == bbb3+r31_exp(:, count)+r32;
%     eqn1 = eval(norm(bbb1+r11_exp(:, count)+r12 - p)) == R_outer_circle_up;
%     eqn2 = eval(norm(bbb2+r21_exp(:, count)+r22 - p)) == R_outer_circle_up;
%     eqn3 = eval(norm(bbb3+r31_exp(:, count)+r32 - p)) == R_outer_circle_up;
% 
% 
% %     sol_1 = solve([eqn1, eqn2, eqn3], [phi_actual_x, phi_actual_y, p_actual_z],'Real',true);
% %     if isempty(sol_1.phi_actual_x)
% %         sol_1 = vpasolve([eqn1, eqn2, eqn3], [phi_actual_x, phi_actual_y, p_actual_z]);
% %     end
%     sol_1 = solve([eqn1, eqn2, eqn3], [phi_actual_x, phi_actual_y, p_actual_z]);
%     if isempty(sol_1.phi_actual_x)
%         sol_1 = vpasolve([eqn1, eqn2, eqn3], [phi_actual_x, phi_actual_y, p_actual_z]);
%     end
% 
%     [R_new_actual_up, p_new_actual_up, phi_new_actual_up] = upper_inverse_kinematic_configuration(sol_1.phi_actual_x(1), sol_1.phi_actual_y(1), sol_1.p_actual_z(1), R_outer_circle_up);
%     R = R_new_actual_up;
%     p = p_new_actual_up;
    r11_a = r11_a_exp(:, count);
    r21_a = r21_a_exp(:, count);
    r31_a = r31_a_exp(:, count);

    % Centre of the upper circle actually
    triangle1 = r11_exp(:, count)+r12+bbb1;
    triangle2 = r21_exp(:, count)+r22+bbb2;
    triangle3 = r31_exp(:, count)+r32+bbb3;
%     [p_new_actual_up, R_new_actual_up] = get_actual_configuration(triangle1, triangle2, triangle3, R_outer_circle_up);
%     R = R_new_actual_up;
%     p = p_new_actual_up;
%     R_new_actual_up_all = [R_new_actual_up_all, R_new_actual_up];
%     p_new_actual_up_all = [p_new_actual_up_all, p_new_actual_up];

    % draw the upper robot
    if mod(count, 40) == 0 || count == COUNT
%         axis equal
%         upper_draw_base_3bar(b_coordinate_source,R_final_down*p, R_final_down*R,c1,c2,c3,c4,c5,c6,'change_color','blue',"coordinate",0);
%         upper_draw_move_3bar(b_coordinate_source, R_final_down*bbb1, R_final_down*bbb2, R_final_down*bbb3, r11_a, r21_a, r31_a, R_final_down*p, R_final_down*R, c11, c22, c33);
        triangle1_a = R_final_down * (r11_exp(:, count)+r12+bbb1) + b_coordinate_source';
        triangle2_a = R_final_down * (r21_exp(:, count)+r22+bbb2) + b_coordinate_source';
        triangle3_a = R_final_down * (r31_exp(:, count)+r32+bbb3) + b_coordinate_source';

        upper_draw_line(triangle1_a,triangle2_a,'change_color','m');
        upper_draw_line(triangle2_a,triangle3_a,'change_color','m');
        upper_draw_line(triangle1_a,triangle3_a,'change_color','m');

        leg11_draw1 = R_final_down * bbb1 + b_coordinate_source';
        leg21_draw1 = R_final_down * bbb2 + b_coordinate_source';
        leg31_draw1 = R_final_down * bbb3 + b_coordinate_source';

        leg11_draw2 = R_final_down * (r11_exp(:, count)+bbb1) + b_coordinate_source';
        leg21_draw2 = R_final_down * (r21_exp(:, count)+bbb2) + b_coordinate_source';
        leg31_draw2 = R_final_down * (r31_exp(:, count)+bbb3) + b_coordinate_source';


        upper_draw_line(leg11_draw1, leg11_draw2);
        upper_draw_line(leg21_draw1, leg21_draw2);
        upper_draw_line(leg31_draw1, leg31_draw2);
        disp("plot");
    end
    
    theta4_old_actual_up = X_cur(1);
    theta5_old_actual_up = X_cur(2);
    theta6_old_actual_up = X_cur(3);
    count = count + 1;
end

% load('R_new_actual_up_all.mat');
% load('p_new_actual_up_all.mat');
% count = 1;
% while count < COUNT
%     if mod(count, 10) == 0
% 
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
%         axis equal
%         upper_draw_base_3bar(b_coordinate_source,R_final_down*p, R_final_down*R,c1,c2,c3,c4,c5,c6,'change_color','blue',"coordinate",0);
%         upper_draw_move_3bar(b_coordinate_source, R_final_down*bbb1, R_final_down*bbb2, R_final_down*bbb3, r11_a_exp(:,end), r21_a_exp(:,end), r31_a_exp(:,end), R_final_down*p, R_final_down*R, c11, c22, c33);
% 
%         R = R_new_actual_up_all(:, (3*(count-1)+1):3*count);
%         p = p_new_actual_up_all(:, count); 
%         r11_a = r11_a_exp(:, count);
%         r21_a = r21_a_exp(:, count);
%         r31_a = r31_a_exp(:, count);
% %         axis equal
% %         upper_draw_base_3bar(b_coordinate_source,R_final_down*p, R_final_down*R,c1,c2,c3,c4,c5,c6,'change_color','blue',"coordinate",0);
% %         upper_draw_move_3bar(b_coordinate_source, R_final_down*bbb1, R_final_down*bbb2, R_final_down*bbb3, r11_a, r21_a, r31_a, R_final_down*p, R_final_down*R, c11, c22, c33);
%         triangle1_a = R_final_down * (r11_exp(:, count)+r12+bbb1) + b_coordinate_source';
%         triangle2_a = R_final_down * (r21_exp(:, count)+r22+bbb2) + b_coordinate_source';
%         triangle3_a = R_final_down * (r31_exp(:, count)+r32+bbb3) + b_coordinate_source';
% 
%         upper_draw_line(triangle1_a,triangle2_a,'change_color','m');
%         upper_draw_line(triangle2_a,triangle3_a,'change_color','m');
%         upper_draw_line(triangle1_a,triangle3_a,'change_color','m');
% 
%         leg11_draw1 = R_final_down * bbb1 + b_coordinate_source';
%         leg21_draw1 = R_final_down * bbb2 + b_coordinate_source';
%         leg31_draw1 = R_final_down * bbb3 + b_coordinate_source';
% 
%         leg11_draw2 = R_final_down * (r11_exp(:, count)+bbb1) + b_coordinate_source';
%         leg21_draw2 = R_final_down * (r21_exp(:, count)+bbb2) + b_coordinate_source';
%         leg31_draw2 = R_final_down * (r31_exp(:, count)+bbb3) + b_coordinate_source';
% 
% 
%         upper_draw_line(leg11_draw1, leg11_draw2);
%         upper_draw_line(leg21_draw1, leg21_draw2);
%         upper_draw_line(leg31_draw1, leg31_draw2);
%         disp("plot");
%     end
%         count = count+1;
% end

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
% 
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
% 
% 












%%
% ---------**************************************************************************************************************---------
% about how to get :
% the forward kinematics answers and are regarded as simulation values


% get configurations during moving using forward kinematic theory
p_actual = p_initial_down;
R_actual = R_initial_down;
phi_actual = phi_initial_down;
d_1y_actual = d_1y_expect;
d_2y_actual = d_2y_expect;
d_3y_actual = d_3y_expect;

% simulative value using forward kinematics
for i = 2:length(Tphi)

 % solve multivariate system of higher-order equations
    syms p_x p_y t c_1x_pre c_1y_pre c_2x_pre c_2y_pre;
    eqn_1 = p_x + (1-t^2)/(1+t^2)*b11(1) - 2*t/(1+t^2)*b11(2) - d_1x_expect(i) - a1(1) == c_1x_pre;
    eqn_2 = p_y + 2*t/(1+t^2)*b11(1) + (1-t^2)/(1+t^2)*b11(2) - d_1y_expect(i) - a1(2) == c_1y_pre;
    eqn_3 =  (c_1x_pre)^2 + (c_1y_pre)^2 == leg_1^2;
    eqn_4 = p_x + (1-t^2)/(1+t^2)*b22(1) - 2*t/(1+t^2)*b22(2) - d_2x_expect(i) - a2(1) == c_2x_pre;
    eqn_5 = p_y + 2*t/(1+t^2)*b22(1) + (1-t^2)/(1+t^2)*b22(2) - d_2y_expect(i) - a2(2) == c_2y_pre;
    eqn_6 =  (c_2x_pre)^2 + (c_2y_pre)^2 == leg_2^2;
    eqn_7 = p_x + (1-t^2)/(1+t^2)*b22(1) - 2*t/(1+t^2)*b22(2) - d_3x_expect(i) - a3(1) == c_2x_pre;
    eqn_8 = p_y + 2*t/(1+t^2)*b22(1) + (1-t^2)/(1+t^2)*b22(2) - d_3y_expect(i) - a3(2) == -c_2y_pre;
    
    eqns = [eqn_1, eqn_2, eqn_3,eqn_4, eqn_5, eqn_6, eqn_7, eqn_8];
    vars = [p_x p_y t c_1x_pre c_1y_pre c_2x_pre c_2y_pre];
    
    % use solve function to get analytical solutions
    sol = solve(eqns,vars,"ReturnConditions",true);
    % use vpasolve function to get numerical solutions
    if isempty(sol.p_x)
        sol = vpasolve(eqns,vars);
    end
    [ans_num, ~] = size(sol.p_x);
    
     % select the index of the first valid value
    for index = 1:ans_num
        if isreal(eval(vpa(sol.p_x(index))))
            num_first = index;
            break;
        end
    end
    
    % select the index of the second valid value
    for index = 1:ans_num
        if isreal(eval(vpa(sol.p_x(index)))) 
            num_second = index;
        end
    end

    % select the value
    num = num_first;
    p_cal_1 = [vpa(sol.p_x(num)),vpa(sol.p_y(num))]';
    phi_cal_1 = double(vpa(2*atan(sol.t(num))/pi*180));
    p_distance_1 = sqrt((p_cal_1(1)-p_actual(1,i-1))^2 + (p_cal_1(2)-p_actual(2,i-1))^2);
    phi_distance_1 = abs(phi_cal_1 - phi_actual(i-1));
    distance_1 = p_distance_1 + phi_distance_1;

    num = num_second;
    p_cal_2 = [vpa(sol.p_x(num)),vpa(sol.p_y(num))]';
    phi_cal_2 = double(vpa(2*atan(sol.t(num))/pi*180));
    p_distance_2 = sqrt((p_cal_2(1)-p_actual(1,i-1))^2 + (p_cal_2(2)-p_actual(2,i-1))^2);
    phi_distance_2 = abs(phi_cal_2 - phi_actual(i-1));
    distance_2 = p_distance_2 + phi_distance_2;

    if distance_1 < distance_2
        num = num_first;
    else
        num = num_second;
    end

    % save the chosen value so as to draw them together
    p_cal = [vpa(sol.p_x(num)),vpa(sol.p_y(num))]';
    R_cal = [cos(vpa(2*atan(sol.t(num)))), -sin(vpa(2*atan(sol.t(num))));sin(vpa(2*atan(sol.t(num)))), cos(vpa(2*atan(sol.t(num))))];
    phi_cal = double(vpa(2*atan(sol.t(num))/pi*180));
     % check whether near the unique point
    [TRUE_OR_FALSE, unique_position_y] = check_limitation(distance,R_cal*b22'+p_cal, a1(2),d_1y_expect(i),leg_1,width);
    if TRUE_OR_FALSE && abs(unique_position_y - d_1y_expect(i)) > distance_L1
        p_actual = [p_actual, p_cal];
        R_actual = [R_actual, R_cal];
        phi_actual = [phi_actual, phi_cal];


        if  i == 2500 || i == length(Tphi)
            draw_base_3bar(a_coordinate_source,'{b}',p_cal,R_cal,b1,b2,b3,b4,b5,b6,len,width,rectangular_W,phi_cal,'change_color','r', "coordinate", 0);
            draw_move_3bar(0,0,0,d_1y_actual(i),d_2y_actual(i),d_3y_actual(i),width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R_cal,p_cal,'change_color','r');
        end

        if mod(i, 100) == 0
                traceA = p_cal+R_cal*b11';
                traceB = p_cal+R_cal*b22';
                plot(traceA(1),traceA(2),    ...
                'Marker','o', ...
                'MarkerSize',1,...
                'MarkerEdgeColor','#EDB120',...
                'MarkerFaceColor','#EDB120');
                plot(traceB(1),traceB(2), ...
                'Marker','o', ...
                'MarkerSize',1,...
                'MarkerEdgeColor','#EDB120',...
                'MarkerFaceColor','#EDB120');
        end

    else
        error('slider1 is out of the limitation!');
    end
end
% ---------**************************************************************************************************************---------

%%

% ************************************************************************************************************************
% set target configuration
% *************************************
phi_target_x_up = 30/180*pi;  % DOF_1
phi_target_y_up = 40/180*pi;  % DOF_2
p_target_z_up = 150;  % DOF_3
% *************************************
[R_target_down, p_target_down, phi_target_down] = upper_inverse_kinematic_configuration(phi_target_x_up, phi_target_y_up, p_target_z_up, R_outer_circle_up);
% ************************************************************************************************************************




% set configuration to trace in frame {b}
t = 0:0.5:500;
s = 0.01*t - 0.1*sin(0.1*pi*t)/pi;
phi_exp_x_up = s;
phi_exp_y_up = pi/5*s;
p_exp_z_up = 20*s + 150;

% maximum iterations
COUNT = size(t);
COUNT = COUNT(2);
% current iterations
count = 1;
phi_old_x_up = phi_initial_up_x;
phi_old_y_up = phi_initial_up_y;
p_old_z_up = p_initial_up_z;
sol_1_old_actual_up = sol_1_initial_up;
sol_2_old_actual_up = sol_2_initial_up;
sol_3_old_actual_up = sol_3_initial_up;
num_1_old_actual_up = num_1_initial_up;
num_2_old_actual_up = num_2_initial_up;
num_3_old_actual_up = num_3_initial_up;

%%
while count <= COUNT
    
    % *************************************
    phi_new_exp_x = phi_exp_x_up(count);  % DOF_1
    phi_new_exp_y = phi_exp_y_up(count);  % DOF_2
    p_new_exp_z = p_exp_z_up(count);  % DOF_3
    % *************************************
    [R_new_exp_up, p_new_exp_up, phi_new_exp_up] = upper_inverse_kinematic_configuration(phi_new_exp_x, phi_new_exp_y, p_new_exp_z, R_outer_circle_up);

    % get initital r and draw the initial configuration of 3D
    % calculate the second rotation joint on the middle in frame {b}
    p = p_new_exp_up;
    R = R_new_exp_up;
    [sol_1_new_exp_up, sol_2_new_exp_up, sol_3_new_exp_up, num_1_new_exp_up, num_2_new_exp_up, num_3_new_exp_up] = upper_inverse_kinematic_solve_equations(...
    p, R, R_outer_circle_down, R_outer_circle_up, leg11, leg12, leg21, leg22, leg31, leg32);
    
    
    % r in frame {b}
    r11 = real(double([leg11*cos(sol_1_new_exp_up.theta1(num_1_new_exp_up)), 0, -leg11*sin(sol_1_new_exp_up.theta1(num_1_new_exp_up))]'));
    r12 = real(double([leg12*cos(sol_1_new_exp_up.theta4(num_1_new_exp_up)), 0, -leg12*sin(sol_1_new_exp_up.theta4(num_1_new_exp_up))]'));
    r21 = real(double([-1/2*leg21*cos(sol_2_new_exp_up.theta2(num_2_new_exp_up)), sqrt(3)/2*leg21*cos(sol_2_new_exp_up.theta2(num_2_new_exp_up)), -leg21*sin(sol_2_new_exp_up.theta2(num_2_new_exp_up))]'));
    r22 = real(double([-1/2*leg22*cos(sol_2_new_exp_up.theta5(num_2_new_exp_up)), sqrt(3)/2*leg22*cos(sol_2_new_exp_up.theta5(num_2_new_exp_up)), -leg22*sin(sol_2_new_exp_up.theta5(num_2_new_exp_up))]'));
    r31 = real(double([-1/2*leg31*cos(sol_3_new_exp_up.theta3(num_3_new_exp_up)), -sqrt(3)/2*leg31*cos(sol_3_new_exp_up.theta3(num_3_new_exp_up)), -leg31*sin(sol_3_new_exp_up.theta3(num_3_new_exp_up))]'));
    r32 = real(double([-1/2*leg32*cos(sol_3_new_exp_up.theta6(num_3_new_exp_up)), -sqrt(3)/2*leg32*cos(sol_3_new_exp_up.theta6(num_3_new_exp_up)), -leg32*sin(sol_3_new_exp_up.theta6(num_3_new_exp_up))]'));
    
    % if unique, break and stop
    Unique = judge_unique(0/180*pi, 120/180*pi, 240/180*pi, R_outer_circle_down, R_outer_circle_up, leg11, leg21, leg31, ...
        sol_1_new_exp_up.theta1(num_1_new_exp_up), sol_2_new_exp_up.theta2(num_2_new_exp_up), sol_3_new_exp_up.theta3(num_3_new_exp_up), double(bbb1 + r11 + r12), double(bbb2 + r21 + r22), double(bbb3 + r31 + r32), ...
        phi_new_exp_up(1), phi_new_exp_up(2), phi_new_exp_up(3));
    if Unique
        disp("UNIQUE CONFIGURATION")
        break;
    end

    % set r in frame {a}
    r11_a = real(double(R_cal*[leg11*cos(sol_1_new_exp_up.theta1(num_1_new_exp_up)), 0, -leg11*sin(sol_1_new_exp_up.theta1(num_1_new_exp_up))]'));
    r21_a = real(double(R_cal*[-1/2*leg21*cos(sol_2_new_exp_up.theta2(num_2_new_exp_up)), sqrt(3)/2*leg21*cos(sol_2_new_exp_up.theta2(num_2_new_exp_up)), -leg21*sin(sol_2_new_exp_up.theta2(num_2_new_exp_up))]'));
    r31_a = real(double(R_cal*[-1/2*leg31*cos(sol_3_new_exp_up.theta3(num_3_new_exp_up)), -sqrt(3)/2*leg31*cos(sol_3_new_exp_up.theta3(num_3_new_exp_up)), -leg31*sin(sol_3_new_exp_up.theta3(num_3_new_exp_up))]'));
    
    % draw the upper robot
    if mod(count, 50) == 0 || count == 1
        axis equal
        upper_draw_base_3bar(b_coordinate_source,R_cal*p, R_cal*R,c1,c2,c3,c4,c5,c6,'change_color','blue');
        upper_draw_move_3bar(b_coordinate_source, R_cal*bbb1, R_cal*bbb2, R_cal*bbb3, r11_a, r21_a, r31_a, R_cal*p, R_cal*R, c11, c22, c33);
    end
    
    
    
    % forward kinematic of upper robot
    syms theta4 theta5 theta6; 
    
    r12 = [leg12*cos(theta4), 0, -leg12*sin(theta4)]';
    r22 = [-1/2*leg22*cos(theta5), sqrt(3)/2*leg22*cos(theta5), -leg22*sin(theta5)]';
    r32 = [-1/2*leg32*cos(theta6), -sqrt(3)/2*leg32*cos(theta6), -leg32*sin(theta6)]';
    
    eqn_1 = eval(norm((bbb1 + r11 + r12)-(bbb2 + r21 + r22)))^2 - 3*R_outer_circle_up*R_outer_circle_up == 0;
    eqn_2 = eval(norm((bbb1 + r11 + r12)-(bbb3 + r31 + r32)))^2 - 3*R_outer_circle_up*R_outer_circle_up == 0;
    eqn_3 = eval(norm((bbb2 + r21 + r22)-(bbb3 + r31 + r32)))^2 - 3*R_outer_circle_up*R_outer_circle_up == 0;
    
    eqns = [eqn_1, eqn_2, eqn_3];
    vars = [theta4 theta5 theta6];
    
    % Newton method
    X_old = [double(sol_1_old_actual_up.theta4(num_1_old_actual_up)), double(sol_2_old_actual_up.theta5(num_2_old_actual_up)), double(sol_3_old_actual_up.theta6(num_3_old_actual_up))]';
    F = [eval(norm((bbb1 + r11 + r12)-(bbb2 + r21 + r22)))^2 - 3*R_outer_circle_up*R_outer_circle_up;
    eval(norm((bbb1 + r11 + r12)-(bbb3 + r31 + r32)))^2 - 3*R_outer_circle_up*R_outer_circle_up;
    eval(norm((bbb2 + r21 + r22)-(bbb3 + r31 + r32)))^2 - 3*R_outer_circle_up*R_outer_circle_up;];
    F_diff = jacobian(F, vars);
    error = 10e-2;
    X_cur = X_old;
    while 1
        F_value = double(subs(F, {theta4 theta5 theta6}, {X_cur(1), X_cur(2), X_cur(3)}));
        if norm(F_value) <= error
            break;
        else
            F_diff_value = double(subs(F_diff, {theta4 theta5 theta6}, {X_cur(1), X_cur(2), X_cur(3)}));
            dk = -eye(3,3)/F_diff_value*F_value;
            X_cur = X_cur + dk;
        end
    end
    
    X_cur
    r12 = [leg12*cos(X_cur(1)), 0, -leg12*sin(X_cur(1))]';
    r22 = [-1/2*leg22*cos(X_cur(2)), sqrt(3)/2*leg22*cos(X_cur(2)), -leg22*sin(X_cur(2))]';
    r32 = [-1/2*leg32*cos(X_cur(3)), -sqrt(3)/2*leg32*cos(X_cur(3)), -leg32*sin(X_cur(3))]';
    
    if mod(count, 50) == 0
        triangle1 = R_cal * (r11+r12+bbb1) + b_coordinate_source';
        triangle2 = R_cal * (r21+r22+bbb2) + b_coordinate_source';
        triangle3 = R_cal * (r31+r32+bbb3) + b_coordinate_source';
        upper_draw_line(triangle1,triangle2,'change_color','m');
        upper_draw_line(triangle2,triangle3,'change_color','m');
        upper_draw_line(triangle1,triangle3,'change_color','m');
        disp("plot");
    end
    
    phi_old_x_up = phi_new_exp_x;
    phi_old_y_up = phi_new_exp_y;
    p_old_z_up = p_new_exp_z;
    sol_1_old_actual_up = sol_1_new_exp_up;
    sol_2_old_actual_up = sol_2_new_exp_up;
    sol_3_old_actual_up = sol_3_new_exp_up;
    num_1_old_actual_up = num_1_new_exp_up;
    num_2_old_actual_up = num_2_new_exp_up;
    num_3_old_actual_up = num_3_new_exp_up;
    count = count + 1;
end
