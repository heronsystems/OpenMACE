%function [XI,X,TP] = myvoronoi(N,TP)
%function [XI,X,TP] = myvoronoi(XI,TP)
function [XI,X,TP] = myvoronoi(args,TP)
global END

% initialize
%initsim

% refreshes persistent variables
clear functions

% dimension of world [x,y]
WORLD = [58 81];

% parse args
XI = [];
if ~nargin, N = 6;
elseif length(args)==1, N = args;
else, XI = args; end

% number of iterations
TIME = 1000;

% maximum xdot per iteration
MAXDOT = 0.5;

% plot tracks
TRACKS = 1;

% x and y grids used for generating surface
tt1 = linspace(-WORLD(1)/2,WORLD(1)/2,WORLD(1));
tt2 = linspace(-WORLD(2)/2,WORLD(2)/2,WORLD(2));
[t1,t2] = meshgrid(tt1,tt2);

% 2D scalar surface
if nargin<2, TP=[]; end
[T,GX,GY,TP] = tworld(t1,t2,WORLD,TP);

% plot surface
figure(gcf), clf, hold on
%set(gcf,'DoubleBuffer','on')
set(gcf,'WindowButtonDownFcn',@mybuttondownfcn)
set(gca,'Linewidth',2,'Fontsize',18), box on
axis xy, axis([tt1(1) tt1(end) tt2(1) tt2(end)]), axis image
map = flipud(colormap('bone')); colormap(map)
global Thandle, Thandle = imagesc(tt1,tt2,T);

% get initial positions from mouse
if isempty(XI),
    for ii=1:N,
        [x,y] = ginput(1);
        if isempty(x) | isempty(y), continue, end
        XI = [XI; x y];
    end
end

% initial from grid
if isempty(XI),
    tmp1 = linspace(tt1(1)+5,tt1(1)+5+min(WORLD)/4,N);
    tmp2 = linspace(tt2(1)+5,tt2(1)+5+min(WORLD)/4,N);
    [x,y] = meshgrid(tmp1,tmp2);
    XI = [x(:) y(:)]; 
end

% set initial positions
X = XI;

% number of vehicles
N = length(X);

% runtime flag
END = 0;

% save bitmap flag
SAVE = 0;

% speed constant
k = .2;
% iterate over time
for ii=1:TIME,
    try, delete(h), catch, end
    
    % reflect the vehicles for voronoi boundaries
    MX = mirror(X,WORLD);
    
    % calculate and plot voronoi
    [VX,VY] = voronoi(MX(:,1),MX(:,2));
    h = plot(VX,VY,'g-','linewidth',2);

    % calculate the voronoi vertices and cells
    [V,C] = voronoin(MX);
    
    % calculate center of mass of each cell
    [CM,M] = calcCM(X,V,C,-T,t1,t2);
    
    % scale and clip the velocity increment
    DX = -k*(X-CM);
    if MAXDOT, DX = DX/norm(DX)*MAXDOT; end
    
    % plot tracks
    g = quiver(X(:,1),X(:,2),DX(:,1),DX(:,2),0);
    set(g,'linewidth',2,'color','b')
    
    % keep the quiver handle for deleting
    if ~TRACKS, h = [h; g]; end
    
    % plot the vehicles
    M = max(12*M/max(M),6);
    for jj=1:N,
        h = [h; plot(X(jj,1),X(jj,2),'ro','linewidth',2, ...
            'markersize',M(jj))];
    end
    axis([tt1(1) tt1(end) tt2(1) tt2(end)])
    drawnow
    if SAVE, saveas(gcf,['figures/f' int2str(ii)],'bmp'), end
    
    % update position
    X = X+DX;
    
    % check runtime flag
    if END, break, end
end
