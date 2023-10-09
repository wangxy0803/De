function [TRUE_OR_FALSE, unique_position_y]= check_limitation(distance,bearing_1, a_y,d_1y_target,leg,width)
% bearing_1 is on the left
% a_y+d_1y_target
% bearing_1(2)-distance*sin(acos(abs(abs(leg-width/2)-abs(bearing_1(1)))/distance))
% a_y+d_1y_target
% useless condition
% bearing_1(2)+(distance+leg)*sin(acos(abs(width/2-bearing_1(1))/(distance+leg)))
if a_y+d_1y_target > bearing_1(2)-distance*sin(acos(abs(abs(leg-width/2)-abs(bearing_1(1)))/distance))
    TRUE_OR_FALSE = true;
else
    TRUE_OR_FALSE = false;
end
unique_position_y = bearing_1(2)+(distance+leg)*sin(acos(abs(width/2-bearing_1(1))/(distance+leg))) - a_y;
end

