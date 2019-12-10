function [nodesXY, LatRef, LongRef] = loadOpenStreetMapSnapToGrid(fileName, refX, refY, boxlength, boxwidth, angle, dx)

% hardcoded parameters
edgeFactor = 1; % dx multiplier for declaring an edge

% parse baseline map
[ways, LatRef, LongRef] = parseOpenStreetMap(fileName);

% generate box
boxXY = boxCorners(refX,refY,boxlength,boxwidth,angle);

% clip, shift to origin, and scale
disp('Manipulating map...')
waysmod = manipOpenStreetMap( ways, boxXY, -angle, 1, 0, 0 );

% resample and snap to grid of width dx
disp('Resampling map...')
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