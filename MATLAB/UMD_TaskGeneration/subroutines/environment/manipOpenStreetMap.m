function waysmod = manipOpenStreetMap( ways, polyXY , angle, scale, shiftX, shiftY )
% rotation matrix
R = [cos(angle) -sin(angle); sin(angle) cos(angle)];
waysmod = [];
k = 1;
for i = 1:1:length(ways)
    % 1) clip to XY polygon
    ind = inpolygon(ways{i}(:,1),ways{i}(:,2),polyXY(:,1), polyXY(:,2));
    if ( any(ind) )
        nodeXYmod = zeros( size( ways{i} ) );
        for j = 1:1:length(nodeXYmod)          
            % 2) rotate then shift
            ptrot = R*(ways{i}(j,:) + [shiftX shiftY])';
            nodeXYmod(j,1) = ptrot(1);
            nodeXYmod(j,2) = ptrot(2);
        end
        % 3) scale resulting nodes
        nodeXYmod(:,1) = scale*nodeXYmod(:,1);
        nodeXYmod(:,2) = scale*nodeXYmod(:,2);        
        waysmod{k} = nodeXYmod;
        k = k + 1;
    end
end




end
