function [local_east, local_north] = F3toENU(eastf3, northf3)


% refX = 333511.4; % m, UTM 18S
% refY = 4315614.5; % m, UTM 18s
% utm_ref = [refX refY]';
pos_ref = [eastf3 northf3]';
angle = -43*pi/180; 
R = [cos(angle) -sin(angle); sin(angle) cos(angle)];

local_east = zeros(size(eastf3 ));
local_north = local_east;
for i = 1:1:length(local_east)
    pos_utm= R*(pos_ref); % + utm_ref;
    local_east(i) = pos_utm(1);
    local_north(i) = pos_utm(2);
end

end
