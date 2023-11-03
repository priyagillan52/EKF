function y= ROTZ(angle)
% function accepts angle in radians
% function returns 3x3 rotation matrix

y = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
end

