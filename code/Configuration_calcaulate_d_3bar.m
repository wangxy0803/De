function [d_x, d_y_1, d_y_2, c_x, c_y_1, c_y_2, succeed] = Configuration_calcaulate_d_3bar(p, phi, b, a, length, INVALID_VALUE, triangle_L, leg)
succeed = 1;
c_x = p(1) + b(1)*cos(phi) - b(2)*sin(phi) - a(1);
if abs(c_x) > leg
    succeed = 0;
end

c_y_1 = sqrt(leg*leg - c_x*c_x);
d_x = 0;
d_y_1 = p(2) + b(1)*sin(phi) + b(2)*cos(phi) - a(2) - c_y_1+1;
if a(2) + d_y_1 + triangle_L/2> length/2 + length
    disp('d_y_1 exceeds the length of trace![Upper]')
    c_y_1 = INVALID_VALUE;
    d_y_1 = INVALID_VALUE;
elseif a(2) + d_y_1  - triangle_L/2< -length/2
    disp('d_y_1 exceeds the length of trace![Lower]')
    c_y_1 = INVALID_VALUE;
    d_y_1 = INVALID_VALUE;
end

c_y_2 = -sqrt(leg*leg - c_x*c_x);
d_x = 0;
d_y_2 = p(2) + b(1)*sin(phi) + b(2)*cos(phi) - a(2) - c_y_2;
if a(2) + d_y_2 > length/2 + length
    disp('d_y_2 exceeds the length of trace![Upper]')
    c_y_2 = INVALID_VALUE;
    d_y_2 = INVALID_VALUE;
elseif a(2) + d_y_2 < -length/2
    disp('d_y_2 exceeds the length of trace![Lower]')
    c_y_2 = INVALID_VALUE;
    d_y_2 = INVALID_VALUE;
end

end
