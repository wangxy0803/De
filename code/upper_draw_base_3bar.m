function [] = upper_draw_base_3bar(b_coordinate_source,p,R,c1,c2,c3,c4,c5,c6,varargin)
% plot(0, 1/4*length, 'r*');
% set coordinate
addition = inputParser;          
addition.addParameter('change_color',0);
addition.addParameter("coordinate",1);
addition.addParameter("transparency",1);
addition.parse(varargin{:}); 
change_color = addition.Results.change_color;
coordinate = addition.Results.coordinate;
transparency = addition.Results.transparency;

axis equal;
if coordinate
    draw_coordinate_system(40,R,[b_coordinate_source(1)+p(1),b_coordinate_source(2)+p(2),b_coordinate_source(3)+p(3)]','rgb', '{c}');
    hold on;
end

% draw hexagon
c1 = R * c1' + b_coordinate_source' + p;
c2 = R * c2' + b_coordinate_source' + p;
c3 = R * c3' + b_coordinate_source' + p;
c4 = R * c4' + b_coordinate_source' + p;
c5 = R * c5' + b_coordinate_source' + p;
c6 = R * c6' + b_coordinate_source' + p;
if change_color
    upper_draw_line(c1,c2,'change_color',change_color,'transparency',transparency);
    upper_draw_line(c2,c3,'change_color',change_color,'transparency',transparency);
    upper_draw_line(c3,c4,'change_color',change_color,'transparency',transparency);
    upper_draw_line(c4,c5,'change_color',change_color,'transparency',transparency);
    upper_draw_line(c5,c6,'change_color',change_color,'transparency',transparency);
    upper_draw_line(c6,c1,'change_color',change_color,'transparency',transparency);
else
    upper_draw_line(c1,c2);
    upper_draw_line(c2,c3);
    upper_draw_line(c3,c4);
    upper_draw_line(c4,c5);
    upper_draw_line(c5,c6);
    upper_draw_line(c6,c1);
end


end