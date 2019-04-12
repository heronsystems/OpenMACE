% function heat2
% 
% Integrates inverse log-likelood ratio tracker with artificial physical
% governing particle motion.
% 
%   TODO distributed LRT
%   TODO performance metrics: e.g., targets detected vs. time
%   TODO heterogeneous agents (mobility, sensing)
%
% Derek Paley, University of Maryland, 2013
function heat2

N = 5;                      % number of agents
M = 3;                       % number of targets
alpha = 0.2;                 % target diffusivity
a = 0.5;                     % gas/liquid param T=a*N+T0
b = 2;                       % solid/liquid param T=b*sqrt(N)+T0
T0 = -15;                    % reference temp and detection threshold
Ksolid = 5;                  % solid-mode gain
Kliquid = 0.05;              % liquid-mode  gain
Kgas = 0.1;                  % gas-mode  gain
PD = 0.95;                   % probability of detection
PF = 0.10;                   % probability of false alarm

n = 35;                      % number of grid points
dx = 2/n;                    % grid spacing
dt = .02;                    % time step
r = alpha^2*dt/2/dx^2;       % intrinsic parameter
numiter = 500;               % number of time steps
grid = linspace(-1,1,n);     % spatial grid

% sensor sensitivity
m = sqrt(2)*erfinv(2*PD-1)-sqrt(2)*erfinv(2*PF-1);

% for plotting
mkf = {'b','w','r'};         % marker face colors
mke = {'r','r','w'};         % marker edge colors

% random seed
rng(1)

% make AX
AX = -2*diag(ones(n^2,1))+diag(ones(n*(n-1),1),n)+diag(ones(n*(n-1),1),-n);

% make AY
Ay = -2*diag(ones(n,1))+diag(ones(n-1,1),1)+diag(ones(n-1,1),-1);
str = [];
for ii=1:n-1,str = [str 'Ay,']; end
str = [str 'Ay'];
eval(['AY = blkdiag(' str ');' ])

% initial temp
[X,Y] = meshgrid(grid);
T = zeros(n,n);

% initial conditions
R = 2*(rand(N,2)-.5);
R0 = 2*(rand(M,2)-.5);

% set up plot
figure(1), clf
hT = imagesc(grid,grid,T,T0*[1 -1]); hold on
set(gca,'fontsize',16)
for jj=1:N,
    h(jj) = plot(R(jj,1),R(jj,2),'ro','markersize',12,'linewidth',2, ...
        'markeredgecolor','r','markerfacecolor','w');
end
ht = plot(R0(:,1),R0(:,2),'wx','markersize',12,'linewidth',2);
hc = colorbar; axis image, axis xy
set(hc,'fontsize',16)
set(get(hc,'ylabel'),'string','Inverse Log-Likelihood Ratio','fontsize',16)
axis([-1 1 -1 1])
xlabel('X','fontsize',16)
ylabel('Y','fontsize',16)
I = eye(n^2);

% iterate over time
numtargets = zeros(numiter,1);
for ii=1:numiter,
      
    % find local temp
    IT = interp2(X,Y,T,R(:,1),R(:,2),'*linear',0);
    IT0 = interp2(X,Y,T,R0(:,1),R0(:,2),'*linear',0);
    
    % move target if it is detected
    jj = find(IT0<T0);
    if ~isempty(jj), numtargets(ii) = length(jj); end
    plot(R0(jj,1),R0(jj,2),'o','markersize',16,'linewidth',2,'markeredgecolor',0.8*[1 1 1])
    plot(R0(jj,1),R0(jj,2),'x','markersize',16,'linewidth',2,'markeredgecolor',0.8*[1 1 1])
    R0(jj,:) = 2*(rand(length(jj),2)-.5);

    % determine state (-1 solid, 0 liquid, 1 gas)
    state = ones(N,1);
    state(find(IT<a*N+T0)) = 0;
    state(find(IT<b*sqrt(N)+T0)) = -1;
    
    % gas mode (normal distribution)
    gas = abs(IT-max(a*N+T0,b*sqrt(N)+T0)).*(state>0)*ones(1,2).*randn(N,2);
        
    % gas mode (Cauchy distribution)
%     gamma = abs(IT-max(a*N+T0,b*sqrt(N)+T0));
%     [gasx,gasy] = pol2cart(2*pi*rand(N,1),gamma.*tan(pi*(rand(N,1)-.5)));
%     gas = (state>0)*ones(1,2).*[gasx gasy];
    
    % liquid mode
    [DTX, DTY] = gradient(T,dx);
    IDTX = interp2(X,Y,DTX,R(:,1),R(:,2),'*linear',0);
    IDTY = interp2(X,Y,DTY,R(:,1),R(:,2),'*linear',0);
    liquid = -[IDTX IDTY];
    
    % solid mode
    DRX = ones(N,1)*R(:,1)'-R(:,1)*ones(1,N);
    DRY = ones(N,1)*R(:,2)'-R(:,2)*ones(1,N);
    DR = sqrt(DRX.^2+DRY.^2)+eps*ones(N,N);
    A = ((DR<1.5*alpha)-eye(N)) & (ones(N,1)*(state'<0));
    tmp = 1-alpha./DR;
    solid = [sum(A.*DRX.*tmp,2) sum(A.*DRY.*tmp,2)];

    % move particles and targets
    R = R+dt*(Kgas*gas+Kliquid*liquid+Ksolid*solid);
    R0 = R0+alpha^2*randn(M,2);
    
    % don't cross walls
    jj = find(abs(R)>1);
    R(jj) = fix(R(jj)); %-10*rem(R(jj),1);
    jj = find(abs(R0)>1);
    R0(jj) = fix(R0(jj));
    
    % compute likelihood ratio
    source = 0;
    for jj=1:N,
        set(h(jj),'xdata',R(jj,1),'ydata',R(jj,2), ...
            'markeredgecolor',mke{state(jj)+2},'markerfacecolor',mkf{state(jj)+2})
        DT = sqrt(sum((ones(M,1)*R(jj,:)-R0).^2,2));
        meas = m*any((DT<alpha))+randn;
        pred = sqrt((R(jj,1)-X).^2+(R(jj,2)-Y).^2) < alpha;
        source = source+pred*m*(meas-.5*m);
    end
    
    % integrate
    T = reshape(((I-r*AX)*(I-r*AY))\((I+r*AX)*((I+r*AY)*T(:))),n,n);
    
    % enforce boundary conditions (zero gradient)
    T(:,1) = T(:,2);
    T(:,end) = T(:,end-1);
    T(1,:) = T(2,:);
    T(end,:) = T(end-1,:);
    
    % update and normalize
    T = T+source/N;
    T = T-mean(mean(T));
    
    % update plot
    set(hT,'cdata',T)
    set(ht,'xdata',R0(:,1),'ydata',R0(:,2))
    drawnow
end

% plot performance
figure(2), clf
plot(1:numiter,cumsum(numtargets))
xlabel('Time step','fontsize',16)
ylabel('Cumulative number of targets detected','fontsize',16)
set(gca,'fontsize',16)
