clc;clear;close all;

% ----------------------------------------------------------------------------------------------------
% underlying robot

% set parameters
INVALID_VALUE = 100000;
a1 = [160, 250];
a2 = [-160, 500];
a3 = [-160, 0];
% radius of hexagon's incircle
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
length = 1000;
a_coordinate_source = [0, 0];
r = 7;
rectangular_L = 60;
rectangular_W = 30;
leg_1 = 250;
leg_2 = 200;
leg_3 = leg_2;
% unique position distance gap
distance_L1 = 10;
% distance of ending leaving
distance_L2 = 80;

% set initial configuration
phi_initial = 0;
p_initial = [0, 0]';
R_initial = [cos(phi_initial/180*pi), -sin(phi_initial/180*pi);sin(phi_initial/180*pi), cos(phi_initial/180*pi)];

% set target configuration
phi_target = 0;
p_target = [0, 0]';
R_target = [cos(phi_target/180*pi), -sin(phi_target/180*pi);sin(phi_target/180*pi), cos(phi_target/180*pi)];

% get initial d and draw the initial configuration
p = p_initial;
phi = phi_initial;
R = R_initial;
disp('get initial d:');
[d_1x, d_1y_1, d_1y_2, c_1x, c_1y_1, c_1y_2] = calcaulate_d_3bar(p, phi/180*pi, b11, a1, 1/2*length, INVALID_VALUE, rectangular_L, leg_1);
disp('calcaulate_d of joint_1 end!');
[d_2x, d_2y_1, d_2y_2, c_2x, c_2y_1, c_2y_2] = calcaulate_d_3bar(p, phi/180*pi, b22, a2, 1/2*length, INVALID_VALUE, rectangular_L, leg_2);
disp('calcaulate_d of joint_2 end!');
[d_3x, d_3y_1, d_3y_2, c_3x, c_3y_1, c_3y_2] = calcaulate_d_3bar(p, phi/180*pi, b22, a3, 1/2*length, INVALID_VALUE, rectangular_L, leg_3);
disp('calcaulate_d of joint_3 end!');
fprintf('\n')

[d_1y,d_2y,d_3y] = select_d(d_1y_1, d_1y_2, d_2y_1, d_2y_2, d_3y_1, d_3y_2, INVALID_VALUE, a2, a3, rectangular_L, 'configuration_1', 'up');
% [TRUE_OR_FALSE, ~] = check_limitation(distance,R_initial*b22'+p_initial, a1(2),d_1y,leg_1,width);
% if TRUE_OR_FALSE
%     draw_base_3bar(a_coordinate_source,p,R,b1,b2,b3,b4,b5,b6,length,width,rectangular_W,phi,'change_color','b');
%     draw_move_3bar(d_1x,d_2x,d_3x,d_1y,d_2y,d_3y,width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R,p,'change_color','b');
% else
%     error('slider1 is out of the limitation!');
% end

% get target d and draw the target configuration
disp('get target d:')
[d_1x_target, d_1y_1_target, d_1y_2_target, c_1x_target, c_1y_1_target, c_1y_2_target] = calcaulate_d_3bar(p_target, phi_target/180*pi, b11, a1, 1/2*length, INVALID_VALUE, rectangular_L, leg_1);
disp('calcaulate_d of joint_1 end!');
[d_2x_target, d_2y_1_target, d_2y_2_target, c_2x_target, c_2y_1_target, c_2y_2_target] = calcaulate_d_3bar(p_target, phi_target/180*pi, b22, a2, 1/2*length, INVALID_VALUE, rectangular_L, leg_2);
disp('calcaulate_d of joint_2 end!');
[d_3x_target, d_3y_1_target, d_3y_2_target, c_3x_target, c_3y_1_target, c_3y_2_target] = calcaulate_d_3bar(p_target, phi_target/180*pi, b22, a3, 1/2*length, INVALID_VALUE, rectangular_L, leg_3);
disp('calcaulate_d of joint_3 end!');
fprintf('\n')


[d_1y_target,d_2y_target,d_3y_target] = select_d(d_1y_1_target, d_1y_2_target, d_2y_1_target, d_2y_2_target, d_3y_1_target, d_3y_2_target, INVALID_VALUE, a2, a3, rectangular_L, 'configuration_1', 'up');
[TRUE_OR_FALSE, ~] = check_limitation(distance,R_target*b22'+p_target, a1(2),d_1y_target,leg_1,width);
if TRUE_OR_FALSE
    draw_base_3bar(a_coordinate_source,'{b}',p_target,R_target,b1,b2,b3,b4,b5,b6,length,width,rectangular_W,phi_target,'change_color','r');
    draw_move_3bar(d_1x_target,d_2x_target,d_3x_target,d_1y_target,d_2y_target,d_3y_target,width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R_target,p_target,'change_color','r');
else
    error('slider1 is out of the limitation!');
end


% get configurations during moving using forward kinematic theory
p_all = p;
R_all = R;
phi_all = phi;
d_1y_all = d_1y;
d_2y_all = d_2y;
d_3y_all = d_3y;


% ************************************************************************************************************************************************
% step1 
% get target configuration and d of step1
count_step1 = 1;
phi_step1 = phi_target;
R_step1 = [cos(phi_step1/180*pi), -sin(phi_step1/180*pi);sin(phi_step1/180*pi), cos(phi_step1/180*pi)];
[p_step1_x, p_step1_y] = get_p_step1(R_target, p_target, b22, R_step1, p_initial, R_initial);
p_step1 = [p_step1_x, p_step1_y]';


disp('get step1 target d:');
[d_1x_step1, d_1y_1_step1, d_1y_2_step1, c_1x_step1, c_1y_1_step1, c_1y_2_step1] = calcaulate_d_3bar(p_step1, phi_step1/180*pi, b11, a1, 1/2*length, INVALID_VALUE, rectangular_L, leg_1);
disp('calcaulate_d of joint_1 end!');
[d_2x_step1, d_2y_1_step1, d_2y_2_step1, c_2x_step1, c_2y_1_step1, c_2y_2_step1] = calcaulate_d_3bar(p_step1, phi_step1/180*pi, b22, a2, 1/2*length, INVALID_VALUE, rectangular_L, leg_2);
disp('calcaulate_d of joint_2 end!');
[d_3x_step1, d_3y_1_step1, d_3y_2_step1, c_3x_step1, c_3y_1_step1, c_3y_2_step1] = calcaulate_d_3bar(p_step1, phi_step1/180*pi, b22, a3, 1/2*length, INVALID_VALUE, rectangular_L, leg_3);
disp('calcaulate_d of joint_3 end!');

[d_1y_step1,d_2y_step1,d_3y_step1] = select_d(d_1y_1_step1, d_1y_2_step1, d_2y_1_step1, d_2y_2_step1, d_3y_1_step1, d_3y_2_step1, INVALID_VALUE, a2, a3, rectangular_L, 'configuration_1', 'up');
[TRUE_OR_FALSE, unique_position_y] = check_limitation(distance,R_step1*b22'+p_step1, a1(2),d_1y_step1,leg_1,width);
if TRUE_OR_FALSE && abs(unique_position_y - d_1y_step1) > distance_L1
    draw_base_3bar(a_coordinate_source,'{b}',p_step1,R_step1,b1,b2,b3,b4,b5,b6,length,width,rectangular_W,phi_step1,'change_color','m');
    draw_move_3bar(d_1x_step1,d_2x_step1,d_3x_step1,d_1y_step1,d_2y_step1,d_3y_step1,width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R_step1,p_step1,'change_color','m');
else
    error('slider1 is out of the limitation!');
end
% -----------------------------------------------------------------------------------------------------------------------
% move horizontally
flag_step1 = 0;
flag_joint1 = 1;
flag_joint2 = 1;
flag_joint3 = 1;
sampling_time_step1 = 1;
velocity_ratio_2_1_step1  = abs(d_2y - d_2y_step1)/abs(d_1y - d_1y_step1);
velocity_ratio_3_1_step1  = abs(d_3y - d_3y_step1)/abs(d_1y - d_1y_step1);
velocity_1_step1 = 50;
velocity_2_step1 = velocity_ratio_2_1_step1*velocity_1_step1;
velocity_3_step1 = velocity_ratio_3_1_step1*velocity_1_step1;
velocity_2_step1 = 5;
velocity_3_step1 = 5;
trend = 1; % slider goes towards the destination when 1 and leave the destination when 0

while 1 
    if flag_step1 == 3
        break;
    end

    count_step1 = int64(count_step1) + 1;
    if trend == 1
        if d_1y > d_1y_step1 
            if d_1y - sampling_time_step1*velocity_1_step1 > d_1y_step1
                d_1y = d_1y - sampling_time_step1*velocity_1_step1;
            else
                d_1y = d_1y_step1;
                flag_step1 = flag_step1 + 1*flag_joint1;
                flag_joint1 = 0;
            end
        else
            if d_1y + sampling_time_step1*velocity_1_step1 < d_1y_step1
                d_1y = d_1y + sampling_time_step1*velocity_1_step1;
            else
                d_1y = d_1y_step1;
                flag_step1 = flag_step1 + 1*flag_joint1;
                flag_joint1 = 0;
            end
        end
    elseif trend == 0
        d_1y = d_1y - sampling_time_step1*velocity_1_step1;
    end


     if d_2y > d_2y_step1 
        if d_2y - sampling_time_step1*velocity_2_step1 > d_2y_step1
            d_2y = d_2y - sampling_time_step1*velocity_2_step1;
        else
            d_2y = d_2y_step1;
            flag_step1 = flag_step1 + 1*flag_joint2;
            flag_joint2 = 0;
        end
    else
        if d_2y + sampling_time_step1*velocity_2_step1 < d_2y_step1
            d_2y = d_2y + sampling_time_step1*velocity_2_step1;
        else
            d_2y = d_2y_step1;
            flag_step1 = flag_step1 + 1*flag_joint2;
            flag_joint2 = 0;
        end
    end

    if d_3y > d_3y_step1
        if d_3y - sampling_time_step1*velocity_3_step1 > d_3y_step1
            d_3y = d_3y - sampling_time_step1*velocity_3_step1;
        else
            d_3y = d_3y_step1;
            flag_step1 = flag_step1 + 1*flag_joint3;
            flag_joint3 = 0;
        end
    else
        if d_3y + sampling_time_step1*velocity_3_step1 < d_3y_step1
            d_3y = d_3y + sampling_time_step1*velocity_3_step1;
        else
            d_3y = d_3y_step1;
            flag_step1 = flag_step1 + 1*flag_joint3;
            flag_joint3 = 0;
        end
    end


    % solve multivariate system of higher-order equations
    syms p_x p_y t c_1x_pre c_1y_pre c_2x_pre c_2y_pre;
    eqn_1 = p_x + (1-t^2)/(1+t^2)*b11(1) - 2*t/(1+t^2)*b11(2) - d_1x - a1(1) == c_1x_pre;
    eqn_2 = p_y + 2*t/(1+t^2)*b11(1) + (1-t^2)/(1+t^2)*b11(2) - d_1y - a1(2) == c_1y_pre;
    eqn_3 =  (c_1x_pre)^2 + (c_1y_pre)^2 == leg_1^2;
    eqn_4 = p_x + (1-t^2)/(1+t^2)*b22(1) - 2*t/(1+t^2)*b22(2) - d_2x - a2(1) == c_2x_pre;
    eqn_5 = p_y + 2*t/(1+t^2)*b22(1) + (1-t^2)/(1+t^2)*b22(2) - d_2y - a2(2) == c_2y_pre;
    eqn_6 =  (c_2x_pre)^2 + (c_2y_pre)^2 == leg_2^2;
    eqn_7 = p_x + (1-t^2)/(1+t^2)*b22(1) - 2*t/(1+t^2)*b22(2) - d_3x - a3(1) == c_2x_pre;
    eqn_8 = p_y + 2*t/(1+t^2)*b22(1) + (1-t^2)/(1+t^2)*b22(2) - d_3y - a3(2) == -c_2y_pre;
    
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
    p_distance_1 = sqrt((p_cal_1(1)-p_all(1,count_step1-1))^2 + (p_cal_1(2)-p_all(2,count_step1-1))^2);
    phi_distance_1 = abs(phi_cal_1 - phi_all(count_step1-1));
    distance_1 = p_distance_1 + phi_distance_1;

    num = num_second;
    p_cal_2 = [vpa(sol.p_x(num)),vpa(sol.p_y(num))]';
    phi_cal_2 = double(vpa(2*atan(sol.t(num))/pi*180));
    p_distance_2 = sqrt((p_cal_2(1)-p_all(1,count_step1-1))^2 + (p_cal_2(2)-p_all(2,count_step1-1))^2);
    phi_distance_2 = abs(phi_cal_2 - phi_all(count_step1-1));
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
    [TRUE_OR_FALSE, unique_position_y] = check_limitation(distance,R_cal*b22'+p_cal, a1(2),d_1y,leg_1,width);
    if TRUE_OR_FALSE && abs(unique_position_y - d_1y) > distance_L1
        p_all = [p_all, p_cal];
        R_all = [R_all, R_cal];
        phi_all = [phi_all, phi_cal];
        d_1y_all = [d_1y_all, d_1y];
        d_2y_all = [d_2y_all, d_2y];
        d_3y_all = [d_3y_all, d_3y];
    
        % draw the figure immediately
        draw_base_3bar(a_coordinate_source,'{b}',p_cal,R_cal,b1,b2,b3,b4,b5,b6,length,width,rectangular_W,phi_cal,'change_color','g');
        draw_move_3bar(d_1x,d_2x,d_3x,d_1y,d_2y,d_3y,width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R_cal,p_cal,'change_color','g');
        pause(1);
        trend = 1;
        velocity_1_step1 = 10;
        velocity_2_step1 = velocity_ratio_2_1_step1*velocity_1_step1;
        velocity_3_step1 = velocity_ratio_3_1_step1*velocity_1_step1;
    elseif TRUE_OR_FALSE && abs(unique_position_y - d_1y) <= distance_L1
        count_step1 = int64(count_step1) - 1;
        velocity_1_step1 = velocity_1_step1*1.5;
        velocity_2_step1 = velocity_2_step1*0.5;
        velocity_3_step1 = velocity_3_step1*0.5;
        trend = 0
    else
        error('slider1 is out of the limitation!');
    end
end



% for i = 2:count_step1
%     draw_base_3bar(a_coordinate_source,p_all(:, i),R_all(1:2, 2*i-1:2*i),b1,b2,b3,b4,b5,b6,length,width,rectangular_W,phi_all(i),'change_color','g');
%     draw_move_3bar(d_1x,d_2x,d_3x,d_1y_all(i),d_2y_all(i),d_3y_all(i),width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R_all(1:2, 2*i-1:2*i),p_all(:,i),'change_color','g');
% end


% ************************************************************************************************************************************************
% step2
p_all = p;
R_all = R;
phi_all = phi;
d_1y_all = d_1y;
d_2y_all = d_2y;
d_3y_all = d_3y;
% get target configuration and d of step2
count_step2= 1;
phi_step2 = phi_target;
R_step2 = [cos(phi_step2/180*pi), -sin(phi_step2/180*pi);sin(phi_step2/180*pi), cos(phi_step2/180*pi)];
[p_step2_x, p_step2_y] = get_p_step2(R_target, p_target, b22, R_step2);
p_step2 = [p_step2_x, p_step2_y]';
T_step2 = [R_step2,p_step2;0 0 1];

disp('get step2 target d:');
[d_1x_step2, d_1y_1_step2, d_1y_2_step2, c_1x_step2, c_1y_1_step2, c_1y_2_step2] = calcaulate_d_3bar(p_step2, phi_step2/180*pi, b11, a1, 1/2*length, INVALID_VALUE, rectangular_L, leg_1);
disp('calcaulate_d of joint_1 end!');
[d_2x_step2, d_2y_1_step2, d_2y_2_step2, c_2x_step2, c_2y_1_step2, c_2y_2_step2] = calcaulate_d_3bar(p_step2, phi_step2/180*pi, b22, a2, 1/2*length, INVALID_VALUE, rectangular_L, leg_2);
disp('calcaulate_d of joint_2 end!');
[d_3x_step2, d_3y_1_step2, d_3y_2_step2, c_3x_step2, c_3y_1_step2, c_3y_2_step2] = calcaulate_d_3bar(p_step2, phi_step2/180*pi, b22, a3, 1/2*length, INVALID_VALUE, rectangular_L, leg_3);
disp('calcaulate_d of joint_3 end!');

[d_1y_step2,d_2y_step2,d_3y_step2] = select_d(d_1y_1_step2, d_1y_2_step2, d_2y_1_step2, d_2y_2_step2, d_3y_1_step2, d_3y_2_step2, INVALID_VALUE, a2, a3, rectangular_L, 'configuration_1', 'up');
[TRUE_OR_FALSE, unique_position_y] = check_limitation(distance,R_step2*b22'+p_step2, a1(2),d_1y_step2,leg_1,width);
if TRUE_OR_FALSE
    draw_base_3bar(a_coordinate_source,'{b}',p_step2,R_step2,b1,b2,b3,b4,b5,b6,length,width,rectangular_W,phi_step2,'change_color','m');
    draw_move_3bar(d_1x_step2,d_2x_step2,d_3x_step2,d_1y_step2,d_2y_step2,d_3y_step2,width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R_step2,p_step2,'change_color','m');
else
    error('slider1 is out of the limitation!');
end

% move vertically
flag_step2 = 0;
sampling_time_step2 = 2;
velocity_ratio_2_1_step2  = abs(d_2y - d_2y_step2)/abs(d_1y - d_1y_step2);
velocity_ratio_3_1_step2  = abs(d_3y - d_3y_step2)/abs(d_1y - d_1y_step2);
velocity_1_step2 = 10;
velocity_2_step2 = velocity_ratio_2_1_step2*velocity_1_step2;
velocity_3_step2 = velocity_ratio_3_1_step2*velocity_1_step2;


while 1 
    if flag_step2 == 1
        break;
    end

    count_step2 = int64(count_step2) + 1;
    if d_1y > d_1y_step2 
        if d_1y - sampling_time_step2*velocity_1_step2 > d_1y_step2
            d_1y = d_1y - sampling_time_step2*velocity_1_step2;
        else
            d_1y = d_1y_step2;
            flag_step2 = 1;
        end
    else
        if d_1y + sampling_time_step2*velocity_1_step2 < d_1y_step2
            d_1y = d_1y + sampling_time_step2*velocity_1_step2;
        else
            d_1y = d_1y_step2;
            flag_step2 = 1;
        end
    end

     if d_2y > d_2y_step2 
        if d_2y - sampling_time_step2*velocity_2_step2 > d_2y_step2
            d_2y = d_2y - sampling_time_step2*velocity_2_step2;
        else
            d_2y = d_2y_step2;
            flag_step2 = 1;
        end
    else
        if d_2y + sampling_time_step2*velocity_2_step2 < d_2y_step2
            d_2y = d_2y + sampling_time_step2*velocity_2_step2;
        else
            d_2y = d_2y_step2;
            flag_step2 = 1;
        end
    end

    if d_3y > d_3y_step2
        if d_3y - sampling_time_step2*velocity_3_step2 > d_3y_step2
            d_3y = d_3y - sampling_time_step2*velocity_3_step2;
        else
            d_3y = d_3y_step2;
            flag_step2 = 1;
        end
    else
        if d_3y + sampling_time_step2*velocity_3_step2 < d_3y_step2
            d_3y = d_3y + sampling_time_step2*velocity_3_step2;
        else
            d_3y = d_3y_step2;
            flag_step2 = 1;
        end
    end
        
        

    % solve multivariate system of higher-order equations
    syms p_x p_y t c_1x_pre c_1y_pre c_2x_pre c_2y_pre c_3x_pre c_3y_pre;
    eqn_1 = p_x + (1-t^2)/(1+t^2)*b11(1) - 2*t/(1+t^2)*b11(2) - d_1x - a1(1) == c_1x_pre;
    eqn_2 = p_y + 2*t/(1+t^2)*b11(1) + (1-t^2)/(1+t^2)*b11(2) - d_1y - a1(2) == c_1y_pre;
    eqn_3 =  (c_1x_pre)^2 + (c_1y_pre)^2 == leg_1^2;
    eqn_4 = p_x + (1-t^2)/(1+t^2)*b22(1) - 2*t/(1+t^2)*b22(2) - d_2x - a2(1) == c_2x_pre;
    eqn_5 = p_y + 2*t/(1+t^2)*b22(1) + (1-t^2)/(1+t^2)*b22(2) - d_2y - a2(2) == c_2y_pre;
    eqn_6 =  (c_2x_pre)^2 + (c_2y_pre)^2 == leg_2^2;
    eqn_7 = p_x + (1-t^2)/(1+t^2)*b22(1) - 2*t/(1+t^2)*b22(2) - d_3x - a3(1) == c_3x_pre;
    eqn_8 = p_y + 2*t/(1+t^2)*b22(1) + (1-t^2)/(1+t^2)*b22(2) - d_3y - a3(2) == c_3y_pre;
    eqn_9 =  (c_3x_pre)^2 + (c_3y_pre)^2 == leg_3^2;
    
    eqns = [eqn_1, eqn_2, eqn_3,eqn_4, eqn_5, eqn_6, eqn_7, eqn_8, eqn_9];
    vars = [p_x p_y t c_1x_pre c_1y_pre c_2x_pre c_2y_pre c_3x_pre c_3y_pre];
   
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
    p_distance_1 = sqrt((p_cal_1(1)-p_all(1,count_step2-1))^2 + (p_cal_1(2)-p_all(2,count_step2-1))^2);
    phi_distance_1 = abs(phi_cal_1 - phi_all(count_step2-1));
    distance_1 = p_distance_1 + phi_distance_1;

    num = num_second;
    p_cal_2 = [vpa(sol.p_x(num)),vpa(sol.p_y(num))]';
    phi_cal_2 = double(vpa(2*atan(sol.t(num))/pi*180));
    p_distance_2 = sqrt((p_cal_2(1)-p_all(1,count_step2-1))^2 + (p_cal_2(2)-p_all(2,count_step2-1))^2);
    phi_distance_2 = abs(phi_cal_2 - phi_all(count_step2-1));
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
    p_all = [p_all, p_cal];
    R_all = [R_all, R_cal];
    phi_all = [phi_all, phi_cal];
    d_1y_all = [d_1y_all, d_1y];
    d_2y_all = [d_2y_all, d_2y];
    d_3y_all = [d_3y_all, d_3y];

    % draw the figure immediately
    draw_base_3bar(a_coordinate_source,'{b}',p_cal,R_cal,b1,b2,b3,b4,b5,b6,length,width,rectangular_W,phi_cal,'change_color','g');
    draw_move_3bar(d_1x,d_2x,d_3x,d_1y,d_2y,d_3y,width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R_cal,p_cal,'change_color','g');
    pause(1);
end


 
% for i = 2:count_step2
%     draw_base_3bar(a_coordinate_source,p_all(:, i),R_all(1:2, 2*i-1:2*i),b1,b2,b3,b4,b5,b6,length,width,rectangular_W,phi_all(i),'change_color','g');
%     draw_move_3bar(d_1x,d_2x,d_3x,d_1y_all(i),d_2y_all(i),d_3y_all(i),width,a1,a2,a3,r,rectangular_L,rectangular_W,b11,b22,R_all(1:2, 2*i-1:2*i),p_all(:,i),'change_color','g');
% end

%%
% ----------------------------------------------------------------------------------------------------
clc;
hold on

% p_cal = [0.75363138351776710218651211860664; 0.066607552994114670347130943216457; 0];
% R_cal = [0.99996243349175437682744438305216, 0.0086678489401179908101516851757452, 0;
%         -0.0086678489401179908101516851757452,  0.99996243349175437682744438305216, 0;
%         0, 0, 1];

p_cal = [p_cal(1); p_cal(2); 0];
R_cal = [R_cal(1,1), R_cal(1,2), 0;
        R_cal(2,1),  R_cal(2,2), 0;
        0, 0, 1];
a_coordinate_source = [0, 0];
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
b_coordinate_source = [a_coordinate_source(1)+p_cal(1),a_coordinate_source(2)+p_cal(2),0];
% set length of legs
leg11 = 200;
leg12 = 200;
leg21 = 200;
leg22 = 200;
leg31 = 200;
leg32 = 200;

% set initial configuration in frame {b}
% *************************************
phi_initial_x = 0/180*pi;  % DOF_1
phi_initial_y = 0/180*pi;  % DOF_2
p_initial_z = 250;  % DOF_3
% *************************************
[R_initial, p_initial, phi_initial] = upper_inverse_kinematic_configuration(phi_initial_x, phi_initial_y, p_initial_z, R_outer_circle_up);


% ************************************************************************************************************************
% set target configuration
% *************************************
phi_target_x = -30/180*pi;  % DOF_1
phi_target_y = 40/180*pi;  % DOF_2
p_target_z = 300;  % DOF_3
% *************************************
[R_target, p_target, phi_target] = upper_inverse_kinematic_configuration(phi_target_x, phi_target_y, p_target_z, R_outer_circle_up);
% ************************************************************************************************************************


% get initital r and draw the initial configuration of 3D
% calculate the second rotation joint on the middle in frame {b}
p = p_initial;
R = R_initial;
[sol_1_initial, sol_2_initial, sol_3_initial, num_1_initial, num_2_initial, num_3_initial] = upper_inverse_kinematic_solve_equations(...
    p, R, R_outer_circle_down, R_outer_circle_up, leg11, leg12, leg21, leg22, leg31, leg32);

% set configuration to trace in frame {b}
t = 0:5:500;
s = 0.005*t - 0.1*sin(0.2*pi*t)/pi;
phi_exp_x = phi_target_x/2.5*s;
phi_exp_y = phi_target_y/2.5*s;
p_exp_z = (p_target_z - p_initial_z)/2.5*s + p_initial_z;

% maximum iterations
COUNT = size(t);
COUNT = COUNT(2);
% current iterations
count = 1;
phi_old_x = phi_initial_x;
phi_old_y = phi_initial_y;
p_old_z = p_initial_z;
sol_1_old = sol_1_initial;
sol_2_old = sol_2_initial;
sol_3_old = sol_3_initial;
num_1_old = num_1_initial;
num_2_old = num_2_initial;
num_3_old = num_3_initial;

% create csv and write labels
fid = fopen('data_01_9.csv', 'w+', 'n'); 
fprintf(fid, '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n', "iterations", "theta_1", "theta_2", "theta_3", "phi_1", "phi_2", "phi_3", "p_x", "p_y", "p_z");

while count <= COUNT
    
    % *************************************
    phi_new_x = phi_exp_x(count);  % DOF_1
    phi_new_y = phi_exp_y(count);  % DOF_2
    p_new_z = p_exp_z(count);  % DOF_3
    % *************************************
    [R_new, p_new, phi_new] = upper_inverse_kinematic_configuration(phi_new_x, phi_new_y, p_new_z, R_outer_circle_up);

    % get initital r and draw the initial configuration of 3D
    % calculate the second rotation joint on the middle in frame {b}
    p = p_new;
    R = R_new;
    [sol_1_new, sol_2_new, sol_3_new, num_1_new, num_2_new, num_3_new] = upper_inverse_kinematic_solve_equations(...
    p, R, R_outer_circle_down, R_outer_circle_up, leg11, leg12, leg21, leg22, leg31, leg32);
    
    
    % r in frame {b}
    r11 = real(double([leg11*cos(sol_1_new.theta1(num_1_new)), 0, -leg11*sin(sol_1_new.theta1(num_1_new))]'));
    r12 = real(double([leg12*cos(sol_1_new.theta4(num_1_new)), 0, -leg12*sin(sol_1_new.theta4(num_1_new))]'));
    r21 = real(double([-1/2*leg21*cos(sol_2_new.theta2(num_2_new)), sqrt(3)/2*leg21*cos(sol_2_new.theta2(num_2_new)), -leg21*sin(sol_2_new.theta2(num_2_new))]'));
    r22 = real(double([-1/2*leg22*cos(sol_2_new.theta5(num_2_new)), sqrt(3)/2*leg22*cos(sol_2_new.theta5(num_2_new)), -leg22*sin(sol_2_new.theta5(num_2_new))]'));
    r31 = real(double([-1/2*leg31*cos(sol_3_new.theta3(num_3_new)), -sqrt(3)/2*leg31*cos(sol_3_new.theta3(num_3_new)), -leg31*sin(sol_3_new.theta3(num_3_new))]'));
    r32 = real(double([-1/2*leg32*cos(sol_3_new.theta6(num_3_new)), -sqrt(3)/2*leg32*cos(sol_3_new.theta6(num_3_new)), -leg32*sin(sol_3_new.theta6(num_3_new))]'));
    
    % if unique, break and stop
    Unique = judge_unique(0/180*pi, 120/180*pi, 240/180*pi, R_outer_circle_down, R_outer_circle_up, leg11, leg21, leg31, ...
        sol_1_new.theta1(num_1_new), sol_2_new.theta2(num_2_new), sol_3_new.theta3(num_3_new), double(bbb1 + r11 + r12), double(bbb2 + r21 + r22), double(bbb3 + r31 + r32), ...
        phi_new(1), phi_new(2), phi_new(3));
    if Unique
        disp("UNIQUE CONFIGURATION")
        break;
    end

    % record in csv
    fprintf(fid, '%d,%f,%f,%f,%f,%f,%f,%f,%f,%f\n', count-1, double(sol_1_new.theta1(num_1_new)), double(sol_2_new.theta2(num_2_new)), ...
        double(sol_3_new.theta3(num_3_new)), phi_new(1), phi_new(2), phi_new(3), p(1), p(2), p(3));
    
    % set r in frame {a}
    r11_a = real(double(R_cal*[leg11*cos(sol_1_new.theta1(num_1_new)), 0, -leg11*sin(sol_1_new.theta1(num_1_new))]'));
    r21_a = real(double(R_cal*[-1/2*leg21*cos(sol_2_new.theta2(num_2_new)), sqrt(3)/2*leg21*cos(sol_2_new.theta2(num_2_new)), -leg21*sin(sol_2_new.theta2(num_2_new))]'));
    r31_a = real(double(R_cal*[-1/2*leg31*cos(sol_3_new.theta3(num_3_new)), -sqrt(3)/2*leg31*cos(sol_3_new.theta3(num_3_new)), -leg31*sin(sol_3_new.theta3(num_3_new))]'));
    
%     % draw the upper robot
%     if mod(count, 50) == 0 || count == 1
%         axis equal
%         upper_draw_base_3bar(b_coordinate_source,R_cal*p, R_cal*R,c1,c2,c3,c4,c5,c6,'change_color','blue');
%         upper_draw_move_3bar(b_coordinate_source, R_cal*bbb1, R_cal*bbb2, R_cal*bbb3, r11_a, r21_a, r31_a, R_cal*p, R_cal*R, c11, c22, c33);
%     end
    
    
    
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
    X_old = [double(sol_1_old.theta4(num_1_old)), double(sol_2_old.theta5(num_2_old)), double(sol_3_old.theta6(num_3_old))]';
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
    
    phi_old_x = phi_new_x;
    phi_old_y = phi_new_y;
    p_old_z = p_new_z;
    sol_1_old = sol_1_new;
    sol_2_old = sol_2_new;
    sol_3_old = sol_3_new;
    num_1_old = num_1_new;
    num_2_old = num_2_new;
    num_3_old = num_3_new;
    count = count + 1;
end












