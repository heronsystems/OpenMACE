% Project a point to a straight line (defined by two points) on a plane
% Example:
% vector = [1,1; 4,4]; % line passing through (1,1) and (4,4)
% q = [3,2]; % point locate at (3,2)
% NOTE: all the points (input and output) are row vectors
% found online at: https://www.mathworks.com/matlabcentral/answers/26464-projecting-a-point-onto-a-line

function [ProjPoint] = pointToLineProjection(vector, q)
p0 = vector(1,:);
p1 = vector(2,:);
a = [-q(1)*(p1(1)-p0(1)) - q(2)*(p1(2)-p0(2)); ...
    -p0(2)*(p1(1)-p0(1)) + p0(1)*(p1(2)-p0(2))]; 
b = [p1(1) - p0(1), p1(2) - p0(2);...
    p0(2) - p1(2), p1(1) - p0(1)];
ProjPoint = -(b\a)';
end