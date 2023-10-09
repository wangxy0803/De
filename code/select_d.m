function [d_1y, d_2y,d_3y] = select_d(d_1y_1, d_1y_2, d_2y_1, d_2y_2, d_3y_1, d_3y_2, INVALID_VALUE, a2, a3, triangle_L,varargin)
parm = inputParser;          
parm.addParameter('configuration_1','down'); 
parm.parse(varargin{:}); 
configuration_1 = parm.Results.configuration_1;
% confirm d_1y_targe
if d_1y_1==INVALID_VALUE && d_1y_2==INVALID_VALUE
    error('d_1y exceeds the length of trace!!');
end
if strcmp(configuration_1,'up')
    if d_1y_1~=INVALID_VALUE && d_1y_2==INVALID_VALUE
        d_1y = d_1y_1;
    elseif d_1y_1==INVALID_VALUE && d_1y_2~=INVALID_VALUE
        d_1y = d_1y_2;
    elseif d_1y_1 > d_1y_2
        d_1y = d_1y_1;
    else
        d_1y = d_1y_2;
    end
elseif strcmp(configuration_1,'down')
    if d_1y_1~=INVALID_VALUE && d_1y_2==INVALID_VALUE
        d_1y = d_1y_1;
    elseif d_1y_1==INVALID_VALUE && d_1y_2~=INVALID_VALUE
        d_1y = d_1y_2;
    elseif d_1y_1 < d_1y_2
        d_1y = d_1y_1;
    else
        d_1y = d_1y_2;
    end
end

% confirm d_2y and d_3y
if d_2y_1==INVALID_VALUE && d_2y_2==INVALID_VALUE
    error('d_2y exceeds the length of trace!!');
end
if d_3y_1==INVALID_VALUE && d_3y_2==INVALID_VALUE
    error('d_3y exceeds the length of trace!!');
end

if d_2y_1 ~= INVALID_VALUE && d_3y_1 ~= INVALID_VALUE && d_2y_1+a2(2)-triangle_L/2 > d_3y_1+a3(2)+triangle_L/2 
    d_2y = d_2y_1;
    d_3y = d_3y_1;
elseif d_2y_1 ~= INVALID_VALUE && d_3y_2 ~= INVALID_VALUE && d_2y_1+a2(2)-triangle_L/2 > d_3y_2+a3(2)+triangle_L/2 
    d_2y = d_2y_1;
    d_3y = d_3y_2;
elseif d_2y_2 ~= INVALID_VALUE && d_3y_1 ~= INVALID_VALUE && d_2y_2+a2(2)-triangle_L/2 > d_3y_1+a3(2)+triangle_L/2 
    d_2y = d_2y_2;
    d_3y = d_3y_1;
elseif d_2y_2 ~= INVALID_VALUE && d_3y_2 ~= INVALID_VALUE && d_2y_2+a2(2)-triangle_L/2 > d_3y_2+a3(2)+triangle_L/2 
    d_2y = d_2y_2;
    d_3y = d_3y_2;
% when bar2 and bar3 are at the same location or collide each other
else
    error("bar2 and bar3 are at the same location!");
end

end

