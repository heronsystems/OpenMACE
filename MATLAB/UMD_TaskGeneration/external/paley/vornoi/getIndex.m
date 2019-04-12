%function Xin = getIndex(X,WORLD)
function [Xin,jj] = getIndex(X,WORLD)

jj = find(X(:,1) < WORLD(1)/2 & X(:,1) > -WORLD(1)/2 & ...
    X(:,2) < WORLD(2)/2 & X(:,2) > -WORLD(2)/2);

%if ~CM,
    Xin = ceil(X(jj,1)+WORLD(1)/2)*WORLD(2)+ceil(X(jj,2)+WORLD(2)/2);
    %else,
   % Xin = (floor(mean(X(jj,1))+WORLD(1)/2)*WORLD(2)+ ...
   %     ceil(mean(X(jj,2))+WORLD(2)/2))*ones(length(jj),1);
   % end