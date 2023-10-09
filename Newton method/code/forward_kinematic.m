close all;
clc;
figure
% draw_base_3bar(a_coordinate_source,p_new,R_sb_new,b1,b2,b3,b4,b5,b6,length,width,triangle_W,phi);
% if d_1y_2 ~= INVALID_VALUE && d_3y_2 ~= INVALID_VALUE && d_4y_1 ~= INVALID_VALUE
%     draw_move_3bar(d_1x,d_3x,d_4x,d_1y_2,d_3y_2,d_4y_1,width,a1,a3,a4,r,triangle_L,triangle_W,b11,b33,b44,R,p);
% end


format long g;
% get valid d
if d_1y_1 ~= INVALID_VALUE
    d_1y = d_1y_1;
else
    d_1y = d_1y_2;
end

if d_3y_1 ~= INVALID_VALUE
    d_3y = d_3y_1;
else
    d_3y = d_3y_2;
end

if d_4y_1 ~= INVALID_VALUE
    d_4y = d_4y_1;
else
    d_4y = d_4y_2;
end

draw_base_3bar(a_coordinate_source,p_new,R_sb_new,b1,b2,b3,b4,b5,b6,length,width,triangle_W,phi);
draw_move_3bar(d_1x,d_3x,d_4x,d_1y,d_3y,d_4y,width,a1,a3,a4,r,triangle_L,triangle_W,b11,b33,b44,R,p);
hold on;



% solve multivariate system of higher-order equations
syms p_x p_y t c_1x_pre c_1y_pre c_3x_pre c_3y_pre c_4x_pre c_4y_pre;
eqn_1 = p_x + (1-t^2)/(1+t^2)*b11(1) - 2*t/(1+t^2)*b11(2) - d_1x - a1(1) == c_1x_pre;
eqn_2 = p_y + 2*t/(1+t^2)*b11(1) + (1-t^2)/(1+t^2)*b11(2) - d_1y - a1(2) == c_1y_pre;
eqn_3 =  (c_1x_pre)^2 + (c_1y_pre)^2 == leg_1^2;
eqn_4 = p_x + (1-t^2)/(1+t^2)*b33(1) - 2*t/(1+t^2)*b33(2) - d_3x - a3(1) == c_3x_pre;
eqn_5 = p_y + 2*t/(1+t^2)*b33(1) + (1-t^2)/(1+t^2)*b33(2) - d_3y - a3(2) == c_3y_pre;
eqn_6 =  (c_3x_pre)^2 + (c_3y_pre)^2 == leg_3^2;
eqn_7 = p_x + (1-t^2)/(1+t^2)*b44(1) - 2*t/(1+t^2)*b44(2) - d_4x - a4(1) == c_4x_pre;
eqn_8 = p_y + 2*t/(1+t^2)*b44(1) + (1-t^2)/(1+t^2)*b44(2) - d_4y - a4(2) == c_4y_pre;
eqn_9 =  (c_4x_pre)^2 + (c_4y_pre)^2 == leg_4^2;

eqns = [eqn_1, eqn_2, eqn_3,eqn_4, eqn_5, eqn_6, eqn_7, eqn_8, eqn_9];
vars = [p_x p_y t c_1x_pre c_1y_pre c_3x_pre c_3y_pre c_4x_pre c_4y_pre];
sol = solve(eqns,vars,"ReturnConditions",true);
% sol = solve(eqns,vars,"ReturnConditions",true,"Real",true);

% index = 0;
% selection = [];
% for cx = 1:6
%     index = cx
%     disp(vpa(sol.c_1x_pre(index)^2+sol.c_1y_pre(index)^2));
%     if vpa(sol.c_1x_pre(index)^2+sol.c_1y_pre(index)^2) == leg^2
%         selection = [selection,index];
%     end
% end


disp('p_x by forward kinematics');
disp(vpa(sol.p_x));
disp('p_y by forward kinematics');
disp(vpa(sol.p_y));
disp('c_1x by forward kinematics');
disp(vpa(sol.c_1x_pre));
disp('c_1y by forward kinematics');
disp(vpa(sol.c_1y_pre));
disp('c_3x by forward kinematics');
disp(vpa(sol.c_3x_pre));
disp('c_3y by forward kinematics');
disp(vpa(sol.c_3y_pre));
disp('c_4x by forward kinematics');
disp(vpa(sol.c_4x_pre));
disp('c_4y by forward kinematics');
disp(vpa(sol.c_4y_pre));
disp('phi by forward kinematics');
disp(vpa(2*atan(sol.t)/pi*180,3));


% if isreal(eval(vpa(sol.p_x(1)))) && abs(eval(vpa(sol.p_x(1))) - p(1)) <= 5
%     num = 1;
% elseif isreal(vpa(sol.p_x(2))) && abs(eval(vpa(sol.p_x(2))) - p(1)) <= 5
%     num = 2;
% elseif isreal(vpa(sol.p_x(3))) && abs(eval(vpa(sol.p_x(3))) - p(1)) <= 5
%     num = 3;
% elseif isreal(vpa(sol.p_x(4))) && abs(eval(vpa(sol.p_x(4))) - p(1)) <= 5
%     num =4;
% elseif isreal(vpa(sol.p_x(5))) && abs(eval(vpa(sol.p_x(5))) - p(1)) <= 5
%     num =5;
% elseif isreal(vpa(sol.p_x(6))) && abs(eval(vpa(sol.p_x(6))) - p(1)) <= 5
%     num =6;
% end

num = 1;
p_cal = [vpa(sol.p_x(num)),vpa(sol.p_y(num))]';
R_cal = [cos(vpa(2*atan(sol.t(num)))), -sin(vpa(2*atan(sol.t(num))));sin(vpa(2*atan(sol.t(num)))), cos(vpa(2*atan(sol.t(num))))];
T_cal = [R_cal,p_cal; 0, 0, 1;];
T_sb_new_cal = T_cal*T_sb;
p_new_cal = T_sb_new_cal(1:2, 3);
R_sb_new_cal = T_sb_new_cal(1:2, 1:2);


draw_base_3bar(a_coordinate_source,p_new_cal,R_sb_new_cal,b1,b2,b3,b4,b5,b6,length,width,triangle_W,double(vpa(2*atan(sol.t(num))/pi*180)));
draw_move_3bar(d_1x,d_3x,d_4x,d_1y,d_3y,d_4y,width,a1,a3,a4,r,triangle_L,triangle_W,b11,b33,b44,R_cal,p_cal);
hold off;


