% refreshes persistent variables
clear functions

% dimension of world [x,y]
WORLD = [58 81];

% parse args
XI = [];
if ~nargin, N = 3;
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