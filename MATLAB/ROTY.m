function y= ROTY(angle)
% function accepts angle in radians
% function returns 3x3 rotation matrix

y = [cos(angle) 0 sin(angle); 0 1 0; -sin(angle) 0 cos(angle)];
end

