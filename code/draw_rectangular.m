function [] = draw_rectangular(o_x, o_y, triangle_L, triangle_W, varargin)
p = inputParser;          
p.addParameter('transparency',1); 
p.parse(varargin{:}); 
transparency = p.Results.transparency;
a = line([o_x-1/2*triangle_W, o_x+1/2*triangle_W], [o_y+1/2*triangle_L, o_y+1/2*triangle_L], 'Linewidth', 2);
b = line([o_x-1/2*triangle_W, o_x+1/2*triangle_W], [o_y-1/2*triangle_L, o_y-1/2*triangle_L], 'Linewidth', 2);
c = line([o_x-1/2*triangle_W, o_x-1/2*triangle_W], [o_y+1/2*triangle_L, o_y-1/2*triangle_L], 'Linewidth', 2);
d = line([o_x+1/2*triangle_W, o_x+1/2*triangle_W], [o_y+1/2*triangle_L, o_y-1/2*triangle_L], 'Linewidth', 2);

a.Color(4) = transparency;
b.Color(4) = transparency;
c.Color(4) = transparency;
d.Color(4) = transparency;
end