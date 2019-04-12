%function [T,GX,GY,TP] = tworld(t1,t2,WORLD,TP)
function [T,GX,GY,TP] = tworld(t1,t2,WORLD,TP)
    persistent maxT
    
% load data?
if 0, [T,GX,GY] = loadWorld(t1,t2,WORLD); return, end
    
if isempty(TP) & size(t1,2)>1,
    NP = ceil(5*abs(randn));
    TP = randn(NP,2);
    TP(:,1) = WORLD(1)/5*TP(:,1);
    TP(:,2) = WORLD(2)/5*TP(:,2);
    TP = [0 0];
end

% random generate peaks
T = zeros(size(t1,1),size(t1,2));
for ii=1:size(TP,1),
    T = T+exp(-((t1+TP(ii,1)).^2)/WORLD(1)^1.3- ...
        ((t2+TP(ii,2)).^2)/WORLD(2)^1.3);
end

% calc gradient and max T
if size(t1,2)>1,
    maxT = max(max(T));
    T = -T/maxT;
    [GX,GY] = gradient(T); 
else
    T = -T/maxT;
end


%%%%%%%%%%%%%%%%
function [out,GX,GY] = loadWorld(t1,t2,WORLD)
    persistent TT, persistent ii, global Thandle
    
out = [];
GX = [];
GY = [];

if size(t1,2)>1,
    if isempty(ii), ii=1; end
    if isempty(TT), load ../data/ICON_aug_01-31-2000_T_w123_surface, end
    T = squeeze(TT(:,:,1,ii));
    [GX,GY] = gradient(T);
    out = T;
else
    out = 0*t1-1;
    [Xin,jj] = getIndex([t1 t2],WORLD);
    T = squeeze(TT(:,:,1,ii));
    set(Thandle,'CData',T)
    out(jj) = T(Xin);
    ii = ii+1;
end
