%%%%%%%%%%%%%%%%%%%%
% reflect positions across the boundaries
function MX = mirror(X,WORLD)
MX(:,1) = [X(:,1); -WORLD(1)-X(:,1); WORLD(1)-X(:,1); X(:,1); X(:,1);];
MX(:,2) = [X(:,2); X(:,2); X(:,2); WORLD(2)-X(:,2); -WORLD(2)-X(:,2)];