function [] = draw_circle(r, d_x, d_y,varargin)
p = inputParser;          
p.addParameter('change_color',0); 
p.addParameter('transparency',1); 
p.parse(varargin{:}); 
change_color = p.Results.change_color;
transparency = p.Results.transparency;
aplha=0:pi/40:2*pi;
x = d_x + r*cos(aplha);
y = d_y + r*sin(aplha);
axis equal
if change_color
    a =  plot(x,y,'Color',change_color);
    a.Color(4) = transparency;
else
    plot(x,y,'Color','b');
end
end