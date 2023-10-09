function [] = draw_move_3bar(d_1x,d_2x,d_3x,d_1y,d_2y,d_3y,width,a1,a2,a3,r,triangle_L,triangle_W,b11,b22,R,p,varargin)
parm = inputParser;          
parm.addParameter('change_color',0); 
parm.addParameter('transparency',1);
parm.parse(varargin{:}); 
change_color = parm.Results.change_color;
transparency = parm.Results.transparency;

% draw roll bearings of Static platform
    o_a1_x = d_1x+1/2*width;
    o_a1_y = a1(2)+d_1y;
    o_a2_x = d_2x-1/2*width;
    o_a2_y = a2(2)+d_2y;
    o_a3_x = d_3x-1/2*width;
    o_a3_y = a3(2)+d_3y;
    draw_circle(r, o_a1_x, o_a1_y, 'change_color',change_color,'transparency',transparency);
    draw_circle(r, o_a2_x, o_a2_y, 'change_color',change_color,'transparency',transparency);
    draw_circle(r, o_a3_x, o_a3_y, 'change_color',change_color,'transparency',transparency);
% draw slider
    draw_rectangular(o_a1_x, o_a1_y, triangle_L, triangle_W,'transparency',transparency);
    draw_rectangular(o_a2_x, o_a2_y, triangle_L, triangle_W,'transparency',transparency);
    draw_rectangular(o_a3_x, o_a3_y, triangle_L, triangle_W,'transparency',transparency);
% draw roll bearings of Dynamic platform
    b11 = R*b11' + p;
    b22 = R*b22' + p;
    if change_color
        draw_circle(r, b11(1), b11(2),'change_color',change_color,'transparency',transparency);
        draw_circle(r, b22(1), b22(2),'change_color',change_color,'transparency',transparency);
    else
        draw_circle(r, b11(1), b11(2));
        draw_circle(r, b22(1), b22(2));
    end
% draw bar
    if change_color
        draw_line(b11, [o_a1_x, o_a1_y],'change_color',change_color,'transparency',transparency);
        draw_line(b22, [o_a2_x, o_a2_y],'change_color',change_color,'transparency',transparency);
        draw_line(b22, [o_a3_x, o_a3_y],'change_color',change_color,'transparency',transparency);
    else
        draw_line(b11, [o_a1_x, o_a1_y]);
        draw_line(b22, [o_a2_x, o_a2_y]);
        draw_line(b22, [o_a3_x, o_a3_y]);
    end
end