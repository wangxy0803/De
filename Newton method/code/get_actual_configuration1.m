function [p_new_actual_up, R_new_actual_up, phix_actual, phiy_actual, phiz_actual] = get_actual_configuration(triangle1, triangle2, triangle3, R_outer_circle_up, c11, c22, c33)
    p_new_actual_up = get_O(triangle1, triangle2, triangle3);
    

    syms phix phiy phiz;
    phi = [phix, phiy, phiz];
    Rx = [1, 0, 0; 
    0, cos(phi(1)), -sin(phi(1)); 
    0, sin(phi(1)), cos(phi(1));];
    Ry = [cos(phi(2)), 0, sin(phi(2)); 
            0, 1, 0; 
            -sin(phi(2)), 0, cos(phi(2));];
    Rz = [cos(phi(3)), -sin(phi(3)), 0; 
            sin(phi(3)), cos(phi(3)), 0; 
            0, 0, 1;];
    R = Rx*Ry*Rz;

    eqn1 = p_new_actual_up + R*c11 == triangle1;
    
    
    sol = solve([eqn1(1), eqn1(2), eqn1(3)], [phix phiy phiz]);
    if isempty(sol.phix)
        sol = vpasolve([eqn1, eqn2, eqn3], [phix phiy phiz]);
    end
    
    phix_actual = mod(double(sol.phix(1)), pi);
    phiy_actual = mod(double(sol.phiy(1)), pi);
    phiz_actual = mod(double(sol.phiz(1)), pi);
    R_new_actual_up = double(subs(R, {phix phiy phiz}, {phix_actual, phiy_actual, phiz_actual}));

end