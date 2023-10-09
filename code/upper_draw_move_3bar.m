function [] = upper_draw_move_3bar(b_coordinate_source, bbb1, bbb2, bbb3, r11, r21, r31, p_ac, R_ac, c11, c22, c33, varargin)
parm = inputParser;          
parm.addParameter('change_color',0); 
parm.addParameter('transparency',0); 
parm.parse(varargin{:}); 
change_color = parm.Results.change_color;
transparency = parm.Results.transparency;

% % draw roll bearings of Dynamic platform
%     b11 = R*b11' + p;
%     b22 = R*b22' + p;
%     if change_color
%         draw_circle(r, b11(1), b11(2),'change_color',change_color);
%         draw_circle(r, b22(1), b22(2),'change_color',change_color);
%     else
%         draw_circle(r, b11(1), b11(2));
%         draw_circle(r, b22(1), b22(2));
%     end
% draw bar
    if change_color
        upper_draw_line(R_ac*c11+b_coordinate_source'+p_ac, b_coordinate_source'+bbb1+r11,'change_color',change_color,'transparency',transparency);
        upper_draw_line(R_ac*c22+b_coordinate_source'+p_ac, b_coordinate_source'+bbb2+r21,'change_color',change_color,'transparency',transparency);
        upper_draw_line(R_ac*c33+b_coordinate_source'+p_ac, b_coordinate_source'+bbb3+r31,'change_color',change_color,'transparency',transparency);
        upper_draw_line(b_coordinate_source'+bbb1, b_coordinate_source'+bbb1+r11,'change_color',change_color,'transparency',transparency);
        upper_draw_line(b_coordinate_source'+bbb2, b_coordinate_source'+bbb2+r21,'change_color',change_color,'transparency',transparency);
        upper_draw_line(b_coordinate_source'+bbb3, b_coordinate_source'+bbb3+r31,'change_color',change_color,'transparency',transparency);
    else
%         some = b_coordinate_source'+bbb1+r11
%         some1 = b_coordinate_source'+p_ac;
%         plot3(some(1),some(2),some(3),"*");
        upper_draw_line(R_ac*c11+b_coordinate_source'+p_ac, b_coordinate_source'+bbb1+r11);
        upper_draw_line(R_ac*c22+b_coordinate_source'+p_ac, b_coordinate_source'+bbb2+r21);
        upper_draw_line(R_ac*c33+b_coordinate_source'+p_ac, b_coordinate_source'+bbb3+r31);
        upper_draw_line(b_coordinate_source'+bbb1, b_coordinate_source'+bbb1+r11);
        upper_draw_line(b_coordinate_source'+bbb2, b_coordinate_source'+bbb2+r21);
        upper_draw_line(b_coordinate_source'+bbb3, b_coordinate_source'+bbb3+r31);
    end
end