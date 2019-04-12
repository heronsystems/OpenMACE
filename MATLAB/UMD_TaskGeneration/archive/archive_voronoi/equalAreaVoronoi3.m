% equalAreaVoronoi

function  [voronoiVertices, voronoiCells, cellMass, cellCenterOfMass] = equalAreaVoronoi3(xx, yy, numCells, xs, ys)

% Based on Algorithm 1: Capacity-Constrained Voronoi Tessellation, in:
% Balzer, Michael, Thomas Schlömer, and Oliver Deussen.
% Capacity-constrained point distributions: a variant of Lloyd's method.
% Vol. 28. No. 3. ACM, 2009.

% Required Inputs:
%   xx : (nrows x ncols) matrix defining domain
%   yy : (nrows x ncols) matrix defining domain
%   numCells : number of desired cells

% Optional Inputs:
%   xs : numCells x 1 vector defining station pts
%   ys : numCells x 1 vector defining station pts
%   Ac : nrows x ncols that maps each point in domain into a station/cell


% A. Wolek, Feb. 2019

% initialize assignment vector
numGridPts = size(xx,1) * size(xx,2);
capacity = floor(numGridPts/numCells);

xmin = xx(1,1);
xmax = xx(1,end);
ymin = yy(1,1);
ymax = yy(end,1);

if (nargin == 3)
    xs = xmin + rand(numCells,1)*(xmax-xmin);
    ys = ymin + rand(numCells,1)*(ymax-ymin);
end
% use random assignment
k = 1;
A = zeros(numGridPts,1);
scap = zeros(numCells,1);
for i = 1:1:size(xx,1)
    for j = 1:1:size(xx,2)
        valid = 0;
        while (valid == 0)
            s = randi(numCells);
            if (scap(s) < capacity)
                valid = 1;
                scap(s) = scap(s) + 1;
            end
        end
        xg(k) = xx(i,j);
        yg(k) = yy(i,j);
        rowInd(k) = i;
        colInd(k) = j;
        A(k) = s;
        Ac(i,j) = s;
        k = k + 1;
    end
end

disp('test')

%
key = zeros(numGridPts,1);
stable = 0;
iter = 0;
while ( stable == 0 )
    stable = 1;
    for i = 1:1:numCells
        for j = 1:1:numCells
            if ( i < j )
                % init two heaps
                Hi = [];
                Hj = [];
                keyi = [];
                keyj = [];
                for k = 1:1:numGridPts
                    xvec = [xg(k) yg(k)];
                    sveci = [xs(i) ys(i)];
                    svecj = [xs(j) ys(j)];
                    if ( A(k) == i )
                        Hi = [Hi k];
                        val = (norm(xvec-sveci))^2 - (norm(xvec-svecj))^2;
                        keyi = [keyi val];
                    end
                    if ( A(k) == j )
                        Hj = [Hj k];
                        val = (norm(xvec-svecj))^2 - (norm(xvec-sveci))^2;
                        keyj = [keyj val];
                    end
                end
                while ( ~isempty(Hi) && ~isempty(Hj) && max(keyi)+max(keyj) > 0 )
                    [maxvali, maxindi] = max(keyi);
                    [maxvalj, maxindj] = max(keyj);
                    
                    A( Hi(maxindi) ) = j;
                    A( Hj(maxindj) ) = i;
                    
                    % remove entries
                    Hi(maxindi) = [];
                    Hj(maxindj) = [];
                    keyi(maxindi) = [];
                    keyj(maxindj) = [];
                    stable = 0;
                end
            end
        end
    end
    iter = iter + 1;
end

% compute the mean for each cell
for i = 1:1:numCells
    assignedPts = find( A == i );
    cellArea(i) = length(assignedPts);
    xbar(i) = mean( xg(assignedPts) );
    ybar(i) = mean( yg(assignedPts) );
end
cellArea
xbar
ybar

% reflect the N coordinates of the vehicles about each map boundary
% (total of four reflections). This is a trick that forces the
% resulting vornoi partition to have boundary cells that conform to the
% rectanglar environment
% Note: MX is (5*N x 2) since we add the four reflections
MX = zeros(5*numCells, 2);
% original points
for i = 1:1:numCells
    X(i,1) = xbar(i);
    X(i,2) = ybar(i);
end
MX(1:numCells,:) = X;
% reflect along left edge
MX(numCells+1:2*numCells,:) = [2*xmin-X(:,1), X(:,2)];
% reflect along right edge
MX(2*numCells+1:3*numCells,:) = [2*xmax-X(:,1), X(:,2)];
% reflect along top edge
MX(3*numCells+1:4*numCells,:) = [X(:,1), 2*ymax-X(:,2)];
% reflect along bottom edge
MX(4*numCells+1:5*numCells,:) = [X(:,1), 2*ymin-X(:,2)];

% calculate the voronoi vertices and cells
[voronoiVertices,voronoiCells] = voronoin(MX);

% calculate center of mass of each cell
cellMass = zeros(numCells,1);
cellCenterOfMass = zeros(numCells,2);
% go through each cell
for j=1:numCells
    VC = voronoiVertices(voronoiCells{j},:);
    % iterate through the grid of (xx,yy) points and find the indices
    % that are inside the voronoi cell
    in = find(inpolygon(xx,yy,VC(:,1),VC(:,2)));
    % sum the mass
    zz = ones(size(xx));
    cellMass(j) = sum(sum(zz(in)));
    % if the mass is very small do not attempt to compute CM
    if abs(cellMass(j))<1e-9
        continue
    end
    cellCenterOfMass(j,1) = sum(sum(zz(in).*xx(in)))/cellMass(j);
    cellCenterOfMass(j,2) = sum(sum(zz(in).*yy(in)))/cellMass(j);
end

