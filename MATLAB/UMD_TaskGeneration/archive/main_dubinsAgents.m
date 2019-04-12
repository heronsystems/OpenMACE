clear; close all; clc;
format compact;

% add paths
addpath('./subroutines')
addpath('./subroutines/geometry')
addpath('./subroutines/graph')
addpath('./subroutines/parsing')
addpath('./subroutines/display')
addpath('./subroutines/controllers')

% Switches
movieFlag = 1;
plotIcFlag = 1;
plotHistoryFlag = 1;
plotPerformanceFlag = 0;

% User parameters
folder = './data/';
fileName = 'martin_small_flat.txt'; % map file from goxel

T = 500; % total simulation time
dt = 0.1; % time-step
borderScale = 1.1;
borderOffset = 20;
Npts = 30;
params.Rsense = 10;
params.Rmin = 3;
params.tol = 0.0;
params.v = 1.0;
N = 8;
params.Ntargs = 10;
vmax = 100;
params.stepSkip = 10;
params.hDiffTol = 5*pi/180;
%params.bounceType = 'normal';
params.bounceType = 'snellsLaw';

%params.bhv.name = 'goStraight';

params.bhv.name = 'followNetwork';
params.bhv.followNetwork.probLock = 1.00; %
params.bhv.followNetwork.probUnLock = 0.00;
params.bhv.followNetwork.refractoryPeriod = 5; %

% derived parameters
params.N = 8;
params.uMax = params.v/params.Rmin;
params.Nsim = floor(T/dt)+1;
maxDistConnectivity = 1;
angOffset = 5*pi/180;

% set up environment and initial conditions for agents
[G,A,nodeX,nodeY,xpoly,ypoly] = loadMap(folder,fileName,borderOffset,maxDistConnectivity);
[xv, yv, prevPt] = distributeUniformlyAlongCurve(params.N,xpoly,ypoly);
[hv] = computeInitialHeading(xv,yv,prevPt,xpoly,ypoly,angOffset);

offset = params.tol*10; % add offset from boundary to prevent 'bounce'
global bhvState;
for i = 1:1:params.N
    x0(3*i-2,1) = xv(i)+offset*cos(hv(i));
    x0(3*i-1,1) = yv(i)+offset*sin(hv(i));
    x0(3*i,1) = hv(i);
    if ( strcmp(params.bhv.name, 'followNetwork') )
        bhvState{i}.status = 'nominal';
        bhvState{i}.refractStartTime = 0.0;
        bhvState{i}.randomHeading = 0.0;
    end
end
x(1,:) = x0';
t(1) = 0;

% pacakge more parameters
params.xpoly = xpoly;
params.ypoly = ypoly;
params.G = G;
params.nodeX = nodeX;
params.nodeY = nodeY;
params.numNodes = length(G);

% initialize target states
for i = 1:1:params.Ntargs
    xt0(4*i-3,1) = randi(params.numNodes);
    curNode = xt0(4*i-3,1);
    randomEdgeIndex = randi( length(G{ curNode }.E) );
    xt0(4*i-2,1) = G{ curNode }.E( randomEdgeIndex );
    xt0(4*i-1,1) = rand()*10 ;
    xt0(4*i,1) = 0.0;
end
xt(1,:) = xt0';

Gm = []; %cell( 1, length(G) );
discoveredNodes = [];
discoveredNodesHist{1} = discoveredNodes;
nodesInViewHist = [];
nodesInViewHist{1} = 0;
for k = 2:1:params.Nsim
    nodesInViewHist{k} = [];
    % simulate searchers
    xdot = swarmDynamicsDubins(t(k-1),x(k-1,:),params);
    x(k,:) = x(k-1,:) + xdot' * dt;
    % simulate discontinuous change in searcher state due to wall bounce
    for i = 1:1:N
        xi = [ x(k,3*i-2); x(k,3*i-1); x(k,3*i)];
        [hitStatus, bounceDir, minDist] = hitsWall(xi,params.xpoly,params.ypoly,params.tol,params.bounceType);
        if ( hitStatus )
            x(k,3*i) = bounceDir;
            if ( strcmp(params.bhv.name, 'followNetwork') )
                bhvState{i}.status = 'nominal';
                bhvState{i}.refractStartTime = [];
                bhvState{i}.randomHeading = [];
            end
        end
    end
    % simulate discovery of graph nodes/update behaviors
    for i = 1:1:N
        xi = [ x(k,3*i-2); x(k,3*i-1); x(k,3*i) ];
        [nodesInView] = nodesInFOV(xi(1),xi(2),params.nodeX,params.nodeY,params.Rsense);
        nodesInViewHist{k} = unique([nodesInViewHist{k} nodesInView]);
        %Gm = updateGraph(Gm, G, A, nodesInView);
        discoveredNodes = updateDiscoveredNodes(discoveredNodes, nodesInView);
        if ( strcmp(params.bhv.name, 'followNetwork') )
            if ( strcmp('following',  bhvState{i}.status) )
                if ( rand() <= params.bhv.followNetwork.probUnLock )
                    bhvState{i}.status = 'refracting';
                    bhvState{i}.refractStartTime = t(k-1);
                    bhvState{i}.randomHeading = rand()*2*pi;
                end
            elseif ( strcmp('nominal',  bhvState{i}.status ) && ~isempty(nodesInView) ) % currently not locked
                if ( rand() <= params.bhv.followNetwork.probLock )
                    bhvState{i}.status = 'following';
                end
            elseif ( strcmp('refracting',  bhvState{i}.status) )
                if ( (t(k-1) - bhvState{i}.refractStartTime) >= params.bhv.followNetwork.refractoryPeriod )
                    bhvState{i}.status = 'nominal';
                end
            end
        end
    end
    % simulate targets
    for i = 1:1:params.Ntargs % xt = [curNode, lastNode, restTime, timeAtNode]
        curNode = xt(k-1,4*i-3);
        lastNode = xt(k-1,4*i-2);
        restTime = xt(k-1,4*i-1);
        timeAtNode = xt(k-1,4*i);
        if ( timeAtNode >= restTime )
            % get list of possible moves
            newNodes =  G{curNode}.E;
            if ( length(newNodes) == 1 )
                lastNode = curNode;
                curNode = G{curNode}.E(1);
                timeAtNode = 0.0;
            else
                % eliminate lastNode from edge set
                ind = find( lastNode == G{curNode}.E );
                newNodes(ind) = [];
                % draw a random new node to visit
                r = randi( length(newNodes) );
                lastNode = curNode;
                curNode = newNodes(r);
                timeAtNode = 0.0;
            end
        else
            timeAtNode = timeAtNode + dt;
        end
        xt(k,4*i-3) = curNode;
        xt(k,4*i-2) = lastNode;
        xt(k,4*i-1) = restTime;
        xt(k,4*i) = timeAtNode;
    end
    %GmHist(k) = Gm;
    discoveredNodesHist{k} = discoveredNodes;
    t(k) = t(k-1) + dt;
    if ( mod(k,floor(params.Nsim/10)) == 0 )
        fprintf('Iter: %d of %d \n', k, params.Nsim)
    end
end

% display flags
% area clearance
minX = min(params.xpoly);
maxX = max(params.xpoly);
minY = min(params.ypoly);
maxY = max(params.ypoly);
% create pixel matrix
params.minXpixel = floor(minX);
params.minYpixel = floor(minY);
params.maxXpixel = ceil(maxX);
params.maxYpixel = ceil(maxY);
params.width = params.maxXpixel - params.minXpixel;
params.height = params.maxYpixel - params.minYpixel;
if ( plotPerformanceFlag )
    figure;
    % Warning calculatePerformance is computationally intensive
    [numNodesDiscovered, numNodesInView, pixelsExplored] = calculatePerformance(t,x,xt,G,discoveredNodesHist,nodesInViewHist,params);
    plotPerformance(t,numNodesDiscovered, pixelsExplored, numNodesInView, params.numNodes, params.width, params.height);
end
if ( plotIcFlag )
    figure;
    plotInitialConditions(xv,yv,hv,params.nodeX, params.nodeY, params.xpoly, params.ypoly);
end
if ( plotHistoryFlag )
    figure;
    plotStateHistory(x,params.nodeX, params.nodeY, params.xpoly, params.ypoly);
end
if ( movieFlag )
    playMovieDubins(t,x,xt,G,discoveredNodesHist,params);
end