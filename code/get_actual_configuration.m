function [p_new_actual_up, R_new_actual_up, phix_actual, phiy_actual, phiz_actual] = get_actual_configuration(triangle1, triangle2, triangle3, R_outer_circle_up, c11, c22, c33)
    p_new_actual_up = get_O(triangle1, triangle2, triangle3);
    
%     syms phix phiy phiz;
%     phi = [phix, phiy, phiz];
%     Rx = [1, 0, 0; 
%     0, cos(phi(1)), -sin(phi(1)); 
%     0, sin(phi(1)), cos(phi(1));];
%     Ry = [cos(phi(2)), 0, sin(phi(2)); 
%             0, 1, 0; 
%             -sin(phi(2)), 0, cos(phi(2));];
%     Rz = [cos(phi(3)), -sin(phi(3)), 0; 
%             sin(phi(3)), cos(phi(3)), 0; 
%             0, 0, 1;];
%     R = Rx*Ry*Rz;
% %     eqn1 = R*c11 + p_new_actual_up== triangle1;
% 
%     eqn1 = p_new_actual_up(1) == R_outer_circle_up*(R(1,1)-R(2,2))/2;
%     eqn2 = p_new_actual_up(2) == -R(2,1)*R_outer_circle_up;
%     eqn3 = phiz == atan2(-sin(phix)*sin(phiy),(cos(phix)+cos(phiy)));
    
%     sol = solve([eqn1, eqn2, eqn3], [phix phiy phiz]);
    fun = @(phi)paramfun(phi,p_new_actual_up,R_outer_circle_up);
    phi0 = [-0.6981, 0.5236, 0.1944];
    phi = fsolve(fun,phi0);
%     sol = fsolve([eqn1, eqn2, eqn3], [phix phiy phiz], [ -0.6981, 0.5236, 0.1944]);
%     sol = solve([eqn1(1), eqn1(2), eqn1(3)], [phix phiy phiz]);
%     if isempty(sol.phix)
%         sol = vpasolve([eqn1, eqn2, eqn3], [phix phiy phiz]);
%     end
    
%     phix_actual = mod(double(sol.phix(1)), pi);
%     phiy_actual = mod(double(sol.phiy(1)), pi);
%     phiz_actual = mod(double(sol.phiz(1)), pi);

%     phix_actual = double(sol.phix(1));
%     phiy_actual = double(sol.phiy(1));
%     phiz_actual = double(sol.phiz(1));
%     R_new_actual_up = double(subs(R, {phix phiy phiz}, {phix_actual, phiy_actual, phiz_actual}));
    phix_actual = phi(1);
    phiy_actual = phi(2);
    phiz_actual = phi(3);

    Rx = [1, 0, 0; 
    0, cos(phi(1)), -sin(phi(1)); 
    0, sin(phi(1)), cos(phi(1));];
    Ry = [cos(phi(2)), 0, sin(phi(2)); 
            0, 1, 0; 
            -sin(phi(2)), 0, cos(phi(2));];
    Rz = [cos(phi(3)), -sin(phi(3)), 0; 
            sin(phi(3)), cos(phi(3)), 0; 
            0, 0, 1;];
    R_new_actual_up = Rx*Ry*Rz;

end