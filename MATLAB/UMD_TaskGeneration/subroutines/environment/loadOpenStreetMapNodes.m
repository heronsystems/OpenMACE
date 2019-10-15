function [nodesXY, LatRef, LongRef] = loadOpenStreetMapNodes(fileName, refX, refY, boxlength, boxwidth, angle, dx, buffer)


edgeFactor = 1; % dx multiplier for declaring an edge


% parse baseline map
[ways, LatRef, LongRef] = parseOpenStreetMap(fileName);

% generate box
boxXY = boxCorners(refX,refY,boxlength,boxwidth,angle);

% clip, shift to origin, and scale
shiftX = -refX;
shiftY = -refY;
waysmod = manipOpenStreetMap( ways, boxXY, -angle, 1, shiftX, shiftY );

% resample and snap to grid of width dx
nodesXY = [];
resampleFactor = 10;
for i = 1:1:length(waysmod)
    [x,y] = resampleCurve( waysmod{i}(:,1), waysmod{i}(:,2), dx/resampleFactor);
    nodesXY = [ nodesXY; x y ];
end
nodesXY(:,1) = round(nodesXY(:,1)./dx)*dx;
nodesXY(:,2) = round(nodesXY(:,2)./dx)*dx;

% remove duplicates
nodesXY = unique(nodesXY,'rows');

%
perimXbuffer = [0 1 1 0 0]*(boxlength-2*buffer)+buffer;
perimYbuffer = [1 1 0 0 1]*(boxwidth-2*buffer)+buffer;

% remove nodes outside the perimeter or inside obstacle
obstacleNode = [];
for i = 1:1:length(nodesXY)
    if ( ~inpolygon(nodesXY(i,1),nodesXY(i,2), perimXbuffer, perimYbuffer) )
        obstacleNode = [obstacleNode i];
    end
end
nodesXY(obstacleNode,:) = [];

% remove edgless nodes
edglessNode = [];
for i = 1:1:length(nodesXY)
    % calculate distance to all other nodes
    dvec = (nodesXY(i,:) - nodesXY);
    d = sqrt(dvec(:,1).^2 + dvec(:,2).^2);
    ind = find(d <= edgeFactor*dx);
    if ( length(ind) == 1 )
        edglessNode = [edglessNode i];
    end
end
nodesXY(edglessNode,:) = [];

end