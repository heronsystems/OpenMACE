function [nodesXY] = loadOpenStreetMapNodes_atF3(fileName, refX, refY, boxlength, angle, buffer, dx, f3Workspace)

% hardcoded parameters
switch f3Workspace
    case 'full'
        f3Width = 26;
        f3Length = (59+28);
        f3LowerLeftCornerX = -59;
        f3LowerLeftCornerY = -13;
    case 'right-square'
        f3Width = 22;
        f3Length = 22;
        f3LowerLeftCornerX = 3;
        f3LowerLeftCornerY = -11;
end
edgeFactor = 1; % dx multiplier for declaring an edge
numPts = 20; %
R = 2;

% derived values
f3Aspect = f3Width/f3Length;
width = boxlength*f3Aspect;

% parse baseline map
[ways, LatRef, LongRef] = parseOpenStreetMap(fileName);

% generate box
boxXY = boxCorners(refX,refY,boxlength,width,angle);

% clip, shift to origin, and scale
scale = f3Length/boxlength;
shiftX = -refX;
shiftY = -refY;
waysmod = manipOpenStreetMap( ways, boxXY, -angle, scale, shiftX, shiftY );

% shift to f3 corner
for i = 1:1:length(waysmod)
    waysmod{i} = waysmod{i} + [f3LowerLeftCornerX f3LowerLeftCornerY];
end

% define obstacles
perimXbuffer = [0 1 1 0 0]*(f3Length-2*buffer)+f3LowerLeftCornerX+buffer;
perimYbuffer = [1 1 0 0 1]*(f3Width-2*buffer)+f3LowerLeftCornerY+buffer;
[pole1x, pole1y] = generateCircle(0, 0, R, numPts);
[pole2x, pole2y] = generateCircle(-30, 0, R, numPts);

% resample and snap to grid of width dx
nodesXY = [];
resampleFactor = 10;
for i = 1:1:length(waysmod)
    [x,y] = resampleCurve( waysmod{i}(:,1), waysmod{i}(:,2), dx/resampleFactor);
    nodesXY = [ nodesXY; x y ];
end
nodesXY(:,1) = round(nodesXY(:,1)./dx)*dx+0.5*dx;
nodesXY(:,2) = round(nodesXY(:,2)./dx)*dx+0.5*dx;

% remove duplicates
nodesXY = unique(nodesXY,'rows');

% remove nodes outside the perimeter or inside obstacle
obstacleNode = [];
for i = 1:1:length(nodesXY)
    if ( inpolygon(nodesXY(i,1),nodesXY(i,2), pole1x, pole1y) || ...
            inpolygon(nodesXY(i,1),nodesXY(i,2), pole2x, pole2y ) || ...
            ~inpolygon(nodesXY(i,1),nodesXY(i,2), perimXbuffer, perimYbuffer) )
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