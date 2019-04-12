function hNormal = normalToLine(x1,y1,x2,y2,dir)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage: hNormal = normalToLine(xA,yA,xB,yB,dir)
%
% Inputs
%   (x1,y1) : first endpoint of line segment
%   (x1,y1) : second endpoint of line segment
%   dir : direction either 'left' or 'right'
%
% Output
%   hNormal : the 'heading' [radians] of the line from pt.1 to pt.2 
%             Heading is measured CCW from the x-axis. Since there are 
%             two possible normal lines, the selection is made according to
%             the 'dir' parameter          
%
% A. Wolek, July 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dx = x2-x1;
dy = y2-y1;
hParallel = atan2(dy,dx);
if ( strcmp(dir,'left') )
	hNormal = mod(hParallel + pi/2, 2*pi);
elseif ( strcmp(dir,'right') )
    hNormal = mod(hParallel - pi/2, 2*pi);  
end
end