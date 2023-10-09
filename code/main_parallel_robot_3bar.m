clear;clc;close all;
% first set parameters
INVALID_VALUE = 100000;
a1 = [160, 250];
% a2 = [160, 388.79];
a3 = [-160, 500];
a4 = [-160, 0];
b1 = [20.21, 75];
b2 = [-20.21, 75];
b3 = [-75.06, -20];
b4 = [-54.85, -55];
b5 = [54.85, -55];
b6 = [75.06, -20];
b11 = [1/2*(b5(1)+b6(1))-20*sin(abs(atan((b6(2)-b5(2))/(b6(1)-b5(1))))), 1/2*(b5(2)+b6(2))+20*cos(abs(atan((b6(2)-b5(2))/(b6(1)-b5(1)))))];
% b22 = [18, 55];
b33 = [0, 55];
b44 = [1/2*(b3(1)+b4(1))+20*sin(abs(atan((b3(2)-b4(2))/(b3(1)-b4(1))))), 1/2*(b3(2)+b4(2))+20*cos(abs(atan((b3(2)-b4(2))/(b3(1)-b4(1)))))];
phi = -10;  % angle value
width = 320;
length = 1000;
a_coordinate_source = [0, 0];
r = 7;
triangle_L = 60;
triangle_W = 30;
R_sb = [1, 0; 0, 1;];
p_sb = [0, 0]';  % initial value p
T_sb = [R_sb, p_sb; 0, 0, 1;];
p = [20, 180]';   % p in T
leg_1 = 200;
leg_3 = 200;
leg_4 = 200;


%second 
% get R p --> T
R = [cos(phi/180*pi), -sin(phi/180*pi);sin(phi/180*pi), cos(phi/180*pi)];
T = [R,p; 0, 0, 1;];
T_sb_new = T*T_sb;
p_new = T_sb_new(1:2, 3);
R_sb_new = T_sb_new(1:2, 1:2);
% z = [0;0];
% R_sb_new = [R_sb_new, z;0,0,1];


% get dx dy by using inverse kinesiology
[d_1x, d_1y_1, d_1y_2, c_1x, c_1y_1, c_1y_2] = calcaulate_d_3bar(p_new, phi/180*pi, b11, a1, 1/2*length, -1, INVALID_VALUE, triangle_L, leg_1);
disp('calcaulate_d of joint_1 end!');
[d_3x, d_3y_1, d_3y_2, c_3x, c_3y_1, c_3y_2] = calcaulate_d_3bar(p_new, phi/180*pi, b33, a3, 1/2*length, 1, INVALID_VALUE, triangle_L, leg_3);
disp('calcaulate_d of joint_3 end!');
[d_4x, d_4y_1, d_4y_2, c_4x, c_4y_1, c_4y_2] = calcaulate_d_3bar(p_new, phi/180*pi, b44, a4, 1/2*length, 0, INVALID_VALUE, triangle_L, leg_4);
disp('calcaulate_d of joint_4 end!');



% draw figures
subplot(2,4,1);
draw_base_3bar(a_coordinate_source,p_new,R_sb_new,b1,b2,b3,b4,b5,b6,length,width,triangle_W,phi);
if d_1y_1 ~= INVALID_VALUE && d_3y_1 ~= INVALID_VALUE && d_4y_1 ~= INVALID_VALUE
    draw_move_3bar(d_1x,d_3x,d_4x,d_1y_1,d_3y_1,d_4y_1,width,a1,a3,a4,r,triangle_L,triangle_W,b11,b33,b44,R,p);
end


subplot(2,4,2);
draw_base_3bar(a_coordinate_source,p_new,R_sb_new,b1,b2,b3,b4,b5,b6,length,width,triangle_W,phi);
if d_1y_1 ~= INVALID_VALUE && d_3y_1 ~= INVALID_VALUE && d_4y_2 ~= INVALID_VALUE
    draw_move_3bar(d_1x,d_3x,d_4x,d_1y_1,d_3y_1,d_4y_2,width,a1,a3,a4,r,triangle_L,triangle_W,b11,b33,b44,R,p);
end


subplot(2,4,3);
draw_base_3bar(a_coordinate_source,p_new,R_sb_new,b1,b2,b3,b4,b5,b6,length,width,triangle_W,phi);
if d_1y_1 ~= INVALID_VALUE && d_3y_2 ~= INVALID_VALUE && d_4y_1 ~= INVALID_VALUE
    draw_move_3bar(d_1x,d_3x,d_4x,d_1y_1,d_3y_2,d_4y_1,width,a1,a3,a4,r,triangle_L,triangle_W,b11,b33,b44,R,p);
end

subplot(2,4,4);
draw_base_3bar(a_coordinate_source,p_new,R_sb_new,b1,b2,b3,b4,b5,b6,length,width,triangle_W,phi);
if d_1y_1 ~= INVALID_VALUE && d_3y_2 ~= INVALID_VALUE && d_4y_2 ~= INVALID_VALUE
    draw_move_3bar(d_1x,d_3x,d_4x,d_1y_1,d_3y_2,d_4y_2,width,a1,a3,a4,r,triangle_L,triangle_W,b11,b33,b44,R,p);
end


subplot(2,4,5);
draw_base_3bar(a_coordinate_source,p_new,R_sb_new,b1,b2,b3,b4,b5,b6,length,width,triangle_W,phi);
if d_1y_2 ~= INVALID_VALUE && d_3y_1 ~= INVALID_VALUE && d_4y_1 ~= INVALID_VALUE
    draw_move_3bar(d_1x,d_3x,d_4x,d_1y_2,d_3y_1,d_4y_1,width,a1,a3,a4,r,triangle_L,triangle_W,b11,b33,b44,R,p);
end


subplot(2,4,6);
draw_base_3bar(a_coordinate_source,p_new,R_sb_new,b1,b2,b3,b4,b5,b6,length,width,triangle_W,phi);
if d_1y_2 ~= INVALID_VALUE && d_3y_1 ~= INVALID_VALUE && d_4y_2 ~= INVALID_VALUE
    draw_move_3bar(d_1x,d_3x,d_4x,d_1y_2,d_3y_1,d_4y_2,width,a1,a3,a4,r,triangle_L,triangle_W,b11,b33,b44,R,p);
end


subplot(2,4,7);
draw_base_3bar(a_coordinate_source,p_new,R_sb_new,b1,b2,b3,b4,b5,b6,length,width,triangle_W,phi);
if d_1y_2 ~= INVALID_VALUE && d_3y_2 ~= INVALID_VALUE && d_4y_1 ~= INVALID_VALUE
    draw_move_3bar(d_1x,d_3x,d_4x,d_1y_2,d_3y_2,d_4y_1,width,a1,a3,a4,r,triangle_L,triangle_W,b11,b33,b44,R,p);
end


subplot(2,4,8);
draw_base_3bar(a_coordinate_source,p_new,R_sb_new,b1,b2,b3,b4,b5,b6,length,width,triangle_W,phi);
if d_1y_2 ~= INVALID_VALUE && d_3y_2 ~= INVALID_VALUE && d_4y_2 ~= INVALID_VALUE
    draw_move_3bar(d_1x,d_3x,d_4x,d_1y_2,d_3y_2,d_4y_2,width,a1,a3,a4,r,triangle_L,triangle_W,b11,b33,b44,R,p);
end



