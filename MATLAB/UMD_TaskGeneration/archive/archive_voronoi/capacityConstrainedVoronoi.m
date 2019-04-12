function  [A, xg, yg, cellArea, Ac, xCM, yCM, cellMass] = capacityConstrainedVoronoi(xx, yy, zz, numCells, xs, ys, plotFlag, Ac)
% Based on Algorithm 1: Capacity-Constrained Voronoi Tessellation appearing
% in:
%
% Balzer, Michael, Thomas Schlömer, and Oliver Deussen.
% Capacity-constrained point distributions: a variant of Lloyd's method.
% Vol. 28. No. 3. ACM, 2009.

if (plotFlag)
    xmin = xx(1,1);
    xmax = xx(1,end);
    ymin = yy(1,1);
    ymax = yy(end,1);
end

% initialize assignment vector
numGridPts = size(xx,1) * size(xx,2);
capacity = floor(numGridPts/numCells);
if (nargin == 7)
    disp('Random Initialization.')
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
elseif (nargin == 8 )
    disp('Using Ac')
    k = 1;
    for i = 1:1:size(xx,1)
    for j = 1:1:size(xx,2)
        xg(k) = xx(i,j);
        yg(k) = yy(i,j);
        rowInd(k) = i;
        colInd(k) = j;
        A(k) = Ac(i,j);
        k = k + 1;        
    end
    end
end

if (plotFlag)
    fig = figure;
    imagesc('XData',[xmin xmax],'YData',[ymin ymax],'CData',Ac)
    set(gca,'YDir','Normal')
    axis equal;
    axis tight;
    colorbar;
end

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
                    
                    
                    % plot debug
                    if (plotFlag)
                        row = rowInd( Hi(maxindi) );
                        col = colInd( Hi(maxindi) );
                        Ac(row,col) = j;
                        row = rowInd( Hj(maxindj) );
                        col = colInd( Hj(maxindj) );
                        Ac(row,col) = i;
                    end
                    
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
    if (plotFlag)
        figure(fig)
        imagesc('XData',[xmin xmax],'YData',[ymin ymax],'CData',Ac)
        set(gca,'YDir','Normal')
        hold off;
        axis equal;
        axis tight;
        drawnow;
    end
end


% compute the mean for each cell
for i = 1:1:numCells
    assignedPts = find( A == i );
    cellArea(i)  = length(assignedPts);
    xbar(i) = mean( xg(assignedPts) );
    ybar(i) = mean( yg(assignedPts) );
end

[xCM, yCM, cellMass] = capacityConstrainedVoronoiCM(xx, yy, zz, Ac, numCells);

if (plotFlag)
    figure(fig)
    imagesc('XData',[xmin xmax],'YData',[ymin ymax],'CData',Ac)
    set(gca,'YDir','Normal')
    hold on;
    axis equal;
    plot(xbar,ybar,'mo','linewidth',2);
    plot(xCM,yCM,'k*','linewidth',2);
    plot(xs,ys,'c+','linewidth',2)
end
