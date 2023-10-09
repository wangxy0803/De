function [] = upper_draw_line(point_a,point_b, varargin)
    p = inputParser;          
    p.addParameter('change_color',0); 
    p.addParameter('transparency',1); 
    p.parse(varargin{:}); 
    change_color = p.Results.change_color;
    transparency = p.Results.transparency;
    if change_color
        a = line([point_a(1), point_b(1)], [point_a(2), point_b(2)], [point_a(3), point_b(3)], 'Color',change_color, 'Linewidth', 2);
        a.Color(4) = transparency;
    else
        line([point_a(1), point_b(1)], [point_a(2), point_b(2)], [point_a(3), point_b(3)], 'Linewidth', 2);
    end
end