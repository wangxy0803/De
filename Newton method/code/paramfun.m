function F = paramfun(phi,p_new_actual_up,R_outer_circle_up)
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
%     eqn1 = R*c11 + p_new_actual_up== triangle1;

    F(1) = p_new_actual_up(1) - (R_outer_circle_up*(R(1,1)-R(2,2))/2);
    F(2) = p_new_actual_up(2) -  (-R(2,1)*R_outer_circle_up);
    F(3) = phi(3) - (atan2(-sin(phi(1))*sin(phi(2)),(cos(phi(1))+cos(phi(2)))));
end