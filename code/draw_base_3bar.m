function [] = draw_base_3bar(a_coordinate_source,name, p_new,R_sb_new,b1,b2,b3,b4,b5,b6,length,width,triangle_W,phi,varargin)

hold on;
p = inputParser;          
p.addParameter("change_color",0); 
p.addParameter("coordinate",1);
p.addParameter('transparency',1);
p.parse(varargin{:}); 
change_color = p.Results.change_color;
coordinate = p.Results.coordinate;
transparency = p.Results.transparency;
% set coordinate
axis equal;
if coordinate
    draw_coordinate_system(20,rotd([0,0,1],phi),[a_coordinate_source(1)+p_new(1),a_coordinate_source(2)+p_new(2),0]','rgb', name);
    hold on;
end

% draw hexagon
b1 = R_sb_new * b1' + p_new;
b2 = R_sb_new * b2' + p_new;
b3 = R_sb_new * b3' + p_new;
b4 = R_sb_new * b4' + p_new;
b5 = R_sb_new * b5' + p_new;
b6 = R_sb_new * b6' + p_new;
if change_color
    draw_line(b1,b2,'change_color',change_color,'transparency',transparency);
    draw_line(b2,b3,'change_color',change_color,'transparency',transparency);
    draw_line(b3,b4,'change_color',change_color,'transparency',transparency);
    draw_line(b4,b5,'change_color',change_color,'transparency',transparency);
    draw_line(b5,b6,'change_color',change_color,'transparency',transparency);
    draw_line(b6,b1,'change_color',change_color,'transparency',transparency);
else
    draw_line(b1,b2);
    draw_line(b2,b3);
    draw_line(b3,b4);
    draw_line(b4,b5);
    draw_line(b5,b6);
    draw_line(b6,b1);
end

% draw moving trace
line([-1/2*width, -1/2*width],[-1/4*length, 3/4*length]);
line([1/2*width, 1/2*width],[-1/4*length, 3/4*length]);
% line([-1/2*width-triangle_W, -1/2*width+triangle_W],[-1/4*length, -1/4*length],'Color','blue','LineWidth',3);
% % line([-1/2*width-triangle_W, -1/2*width+triangle_W],[1/4*length, 1/4*length],'Color','blue','LineWidth',3);
% line([-1/2*width-triangle_W, -1/2*width+triangle_W],[3/4*length, 3/4*length],'Color','blue','LineWidth',3);
% line([1/2*width-triangle_W, 1/2*width+triangle_W],[-1/4*length, -1/4*length],'Color','blue','LineWidth',3);
% line([1/2*width-triangle_W, 1/2*width+triangle_W],[3/4*length, 3/4*length],'Color','blue','LineWidth',3);

% plot(160, 250, 'r*');
% plot(-160, 500, 'r*');
% plot(-160, 0, 'r*');

end