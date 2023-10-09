function [p_step1_x, p_step1_y] = get_p_step1(R_target, p_target, b22, R_step1, p_initial, R_initial)
% p_step1_y = p_initial(2);

% p_step1_y makes b22's y consistant during step1
syms p__y;
eqn = R_initial(2,:)*b22'+ p_initial(2) ==  R_step1(2,:)*b22'+p__y;
sol = solve(eqn,p__y,"ReturnConditions",true);
p_step1_y = eval(vpa(sol.p__y));

[x,~] = get_p_step2(R_target, p_target, b22, R_step1);
p_step1_x = eval(vpa(x));
end

