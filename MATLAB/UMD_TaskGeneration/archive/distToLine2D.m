function [dist, angle] = distToLine2D(a, b, c, x0, y0)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [dist] = distToLine2D(a, b, c, x0, y0)
%   Returns the (unsigned) distance from the pt (x0, y0) to a line defined
%   by an equation in standard form: ax + by + c = 0
%
% Inputs: (x0,y0) : the point from which the distance is calculated
%         (a,b,c) : coefficients of the line
% Outputs: dist : an (unsigned) distance defined by the perpendicular to
%                 the line that passes through the point (x0,y0)
%          angle : from (x0,y0) pt to the nearest point on the line,
%                  measured CCW from the x-axis
% Artur Wolek                                    Last Modified: July-2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (b == 0 && a ~= 0)% the line eq. is x = -c/a (vertical line)
    dist = x0 - (-c/a);
    if dist > 0
        angle = 0; % pt is to right
    else
        angle = pi; % pt is to the left
    end
    dist = abs(dist);
    
elseif ( b ~=0 && a == 0 )% the line eq. is y = -c/b (horizontal line)
	dist = y0 - (-c/b);
    if dist > 0 
        angle = pi/2; % pt is above
    else
        angle = 3*pi/2; % pt is below
    end
    dist = abs(dist);
    
else
    dist = abs(a*x0+b*y0+c)/sqrt(a.^2 + b.^2);
    
    % Pt. A ) define a ref. pt. along the line
    xa = x0;
    ya = -a/b*xa-c/b;
    
    % Pt. B ) define another pt. along the line, 1 unit away along the x-axis
    xb = xa + 1;
    yb = -a/b*xb-c/b;
    
    % define vector pointing from A to B
    vec1 = [xb-xa yb-ya]';
    vec2 = [x0-xa y0-ya]';
    
    % the determinant gives the "signed area" of the parallelogram
    signedArea = vec1(1)*vec2(2) - vec1(2)*vec2(1);
    if signedArea > 0 % pt is to the left of the line
        angle = mod(atan(b/a),2*pi);
    else % pt is to the right of the line
        angle = mod(atan(b/a)+pi,2*pi);
    end
end

end
