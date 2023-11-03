function y= ROTX(angle)
% function accepts angle in radians
% function returns 3x3 rotation matrix

y = [1 0 0; 0 cos(angle) -sin(angle); 0 sin(angle) cos(angle)];
end

