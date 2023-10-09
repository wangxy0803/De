function [O] = get_O(triangle1, triangle2, triangle3)
A1 = triangle1(2)*triangle2(3) - triangle1(2)*triangle3(3) - triangle1(3)*triangle2(2) + triangle1(3)*triangle3(2) + triangle2(2)*triangle3(3) - triangle3(2)*triangle2(3);
B1 = -triangle1(1)*triangle2(3) + triangle1(1)*triangle3(3) + triangle1(3)*triangle2(1) - triangle1(3)*triangle3(1) - triangle2(1)*triangle3(3) + triangle3(1)*triangle2(3);
C1 = triangle1(1)*triangle2(2) - triangle1(1)*triangle3(2) - triangle1(2)*triangle2(1) + triangle1(2)*triangle3(1) + triangle2(1)*triangle3(2) - triangle3(1)*triangle2(2);
D1 = -triangle1(1)*triangle2(2)*triangle3(3) + triangle1(1)*triangle3(2)*triangle2(3) + triangle2(1)*triangle1(2)*triangle3(3) - triangle3(1)*triangle1(2)*triangle2(3) - triangle2(1)*triangle3(2)*triangle1(3) + triangle3(1)*triangle2(2)*triangle1(3);

A2 = 2*(triangle2(1)-triangle1(1));
B2 = 2*(triangle2(2)-triangle1(2));
C2 = 2*(triangle2(3)-triangle1(3));
D2= triangle1(1)^2+triangle1(2)^2+triangle1(3)^2-triangle2(1)^2-triangle2(2)^2-triangle2(3)^2;

A3 = 2*(triangle3(1)-triangle1(1));
B3 = 2*(triangle3(2)-triangle1(2));
C3 = 2*(triangle3(3)-triangle1(3));
D3 = triangle1(1)^2+triangle1(2)^2+triangle1(3)^2-triangle3(1)^2-triangle3(2)^2-triangle3(3)^2;


O = -inv([A1, B1, C1;A2, B2, C2;A3, B3, C3;])*[D1, D2, D3]';

% hold on;
% line([triangle1(1), triangle2(1)], [triangle1(2), triangle2(2)], [triangle1(3), triangle2(3)]);
% line([triangle1(1), triangle3(1)], [triangle1(2), triangle3(2)], [triangle1(3), triangle3(3)]);
% line([triangle2(1), triangle3(1)], [triangle2(2), triangle3(2)], [triangle2(3), triangle3(3)]);
% plot3(O(1),O(2),O(3), "*");

% norm(O-triangle1')
% norm(O-triangle2')
% norm(O-triangle3')
end