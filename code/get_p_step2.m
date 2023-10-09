function [x, y] = get_p_step2(R_target, p_target, b22, R_step)
syms p__x p__y;
eqn = R_target*b22'+p_target ==  R_step*b22'+[p__x,p__y]';
sol = solve(eqn,[p__x p__y],"ReturnConditions",true);
x = eval(vpa(sol.p__x));
y = eval(vpa(sol.p__y));
return

