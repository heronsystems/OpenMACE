function [xf3, yf3] = UTMtoF3(xutm, yutm)

% first pole directly in front of canopy 
refX = 333511.4; % m, UTM 18S
refY = 4315614.5; % m, UTM 18s
pos_ref = [refX refY]';
% [Lat,Lon] = utm2deg(refX,refY,'18 S')
% Lat =
%   38.973698589360254
% Lon =
%  -76.921896771622713


% units: m, in non-rotated but F3 origin referenced frame
%westCornerX = -54.337873554780316; 
%westCornerY = 31.808160981662596; 
%southCornerX = 9.443386236902668;
%southCornerY = -26.567907302250646;
%delx = westCornerX - southCornerX;
%dely = westCornerY - southCornerY;

angle = 43*pi/180; %atan2(delx,dely);
R = [cos(angle) -sin(angle); sin(angle) cos(angle)];

xf3 = zeros(size(xutm));
yf3 = xf3;

for i = 1:1:length(xutm)
    pos_utm = [xutm(i) yutm(i)]';
    pos_f3 = R*(pos_utm - pos_ref);
    xf3(i) = pos_f3(1);
    yf3(i) = pos_f3(2);
end

end
