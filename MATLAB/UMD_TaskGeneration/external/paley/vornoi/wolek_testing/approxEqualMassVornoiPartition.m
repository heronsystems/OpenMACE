%function [voronoiVertices, voronoiCells, cellMass, cellCenterOfMass] = approxEqualMassVornoiPartition(xx,yy,zz,N,stepSizeGain,percentTol,maxIters)
%
% Description: Decomposes the surface (xx,yy,zz) into N number of
% approximately equal mass cells.
%
% Input:
%   (xx,yy,zz) : describes a surface with domain (xx,yy) and values (zz).
%       all three entries are matrices of the same size (e.g., xx, yy are
%       produced by meshgrid)
%   N : number of tasks/cells to decompose surface into
%   stepSizeGain: feedback gain for the iterative algorithm (typical value
%       is 0.2)
%   percentTol : termination condition for stopping the algorithm when the
%       mass of each cell are approximately equal to within this percent
%       tolerance.
%   maxIters : a secondary termination condition
%
% Output:
%   voronoiVertices : matrix with two columns giving x,y coordinates of
%       each vertex
%   voronoiCells : cell array of length N, each cell gives array of
%       vertices defining that cell
%   cellMass : (N x 1) vector of mass of each cell
%   cellCenterOfMass : (N x 2) matrix of coordinates for each cell center
%       of mass
%
% Artur Wolek, Sep 2018
% Adapted from 'function [XI,X,TP] = myvoronoi(XI,TP)' by Derek Paley

function [voronoiVertices, voronoiCells, cellMass, cellCenterOfMass] = approxEqualMassVornoiPartition(xx,yy,zz,N,stepSizeGain,percentTol,maxIters)

% world dimensions
minX = min(min(xx));
maxX = max(max(xx));
minY = min(min(yy));
maxY = max(max(yy));

% variable used to determine convergence, quantifies how much the worst
% case cell weight deviates from the mean
maxOffsetSqPercent = Inf;

% generate random start points
XI = zeros(N,2);
for i = 1:1:N
    XI(i,1) = minX + rand()*(maxX - minX);
    XI(i,2) = minY + rand()*(maxY - minY);
end

% iterate until maxIters is reached or threshold percent is reached
i = 1;
X = XI;
while ( maxOffsetSqPercent > percentTol && i < maxIters )
    
    % reflect the N coordinates of the vehicles about each map boundary
    % (total of four reflections). This is a trick that forces the
    % resulting vornoi partition to have boundary cells that conform to the
    % rectanglar environment
    % Note: MX is (5*N x 2) since we add the four reflections    
    MX = zeros(5*N, 2);   
    % original points
    MX(1:N,:) = X;
    % reflect along left edge
    MX(N+1:2*N,:) = [2*minX-X(:,1), X(:,2)];
    % reflect along right edge
    MX(2*N+1:3*N,:) = [2*maxX-X(:,1), X(:,2)];
    % reflect along top edge
    MX(3*N+1:4*N,:) = [X(:,1), 2*maxY-X(:,2)];
    % reflect along bottom edge
    MX(4*N+1:5*N,:) = [X(:,1), 2*minY-X(:,2)];
   
    % calculate the voronoi vertices and cells
    [voronoiVertices,voronoiCells] = voronoin(MX);
    
    % calculate center of mass of each cell
    cellMass = zeros(N,1);
    cellCenterOfMass = zeros(N,2);
    % go through each cell
    for j=1:N
        VC = voronoiVertices(voronoiCells{j},:);
        % iterate through the grid of (xx,yy) points and find the indices
        % that are inside the voronoi cell
        in = find(inpolygon(xx,yy,VC(:,1),VC(:,2)));
        % sum the mass
        cellMass(j) = sum(sum(zz(in)));
        % if the mass is very small do not attempt to compute CM
        if abs(zz(j))<1e-9
            continue
        end
        cellCenterOfMass(j,1) = sum(sum(zz(in).*xx(in)))/cellMass(j);
        cellCenterOfMass(j,2) = sum(sum(zz(in).*yy(in)))/cellMass(j);
    end
    
    % calculate worst case deviation from mean
    meanM = mean(cellMass);
    maxOffsetSqPercent = max((cellMass - meanM).^2 / meanM^2)/N;
    
    % move each vehicle towards the center of mass
    % scale the velocity increment
    DX = -stepSizeGain*(X-cellCenterOfMass);
    
    %     % clip the velocity increment
    %     DX = DX/norm(DX)*maxStepSize;
    
    % update position
    X = X+DX;
    
    % update index
    i = i + 1;
end

if ( i >= maxIters )
   disp('approxEqualMassVornoiPartition: Warning, terminated due to max iterations.'); 
end


end