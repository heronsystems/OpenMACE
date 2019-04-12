%function [XI,X,TP] = mysim(N,TP)
%function [XI,X,TP] = mysim(XI,TP)
function [XI,X,TP] = mysim(args,TP)
global END

% initialize
initsim

% scalar weighting [kT kG], if kT<0 then asymmetric
k = [2 .1]/length(X);

% equilibrium range
R0 = 12;

% iterate over time
for ii=1:TIME,
    
    % calc xdot
    Tx = tworld(X(:,1),X(:,2),WORLD,TP);
    A = calcA(X,Tx,R0,WORLD,k);
    XDOT = A*X;
    if MAXDOT, XDOT = XDOT/norm(XDOT)*MAXDOT; end
    
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