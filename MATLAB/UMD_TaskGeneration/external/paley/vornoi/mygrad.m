%function [XI,X,TP] = mysim(N,TP)
%function [XI,X,TP] = mysim(XI,TP)
function [XI,X,TP] = mysim(args,TP)
global END

% initialize
initsim

% scalar weighting [kGrad kG]
k = [1 2]/length(X);

% equilibrium range
R0 = 12;

% iterate over time
for ii=1:TIME,
    
    % calc xdot
    Tx = tworld(X(:,1),X(:,2),WORLD,TP);
    A = calcA(X,Tx,R0,WORLD,[0 k(2)]);
    XDOT = A*X;
    if MAXDOT, XDOT = XDOT/norm(XDOT)*MAXDOT; end
    
    % get local gradient
    [Xin,jj] = getIndex(X,WORLD);
    XDOT(jj,:) = XDOT(jj,:)-k(1)*[WORLD(1)*GX(Xin) WORLD(2)*GY(Xin)];
    
    % plot
    h = plot(X(:,1),X(:,2),'ro','linewidth',2,'markersize',8);
    g = quiver(X(:,1),X(:,2),XDOT(:,1),XDOT(:,2),0);
    set(g,'linewidth',2,'color','b')
    if ~TRACKS, h = [h; g]; end
    drawnow
    if SAVE, saveas(gcf,['figures/f' int2str(ii)],'bmp'), end
    try, delete(h), catch, end
        
    % update the positions
    X = X+XDOT;
    
    % check runtime flag
    if END, break, end
end

% plot
endsim

