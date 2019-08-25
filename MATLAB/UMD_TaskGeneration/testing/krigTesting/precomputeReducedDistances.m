function E = precomputeReducedDistances(xcp, ycp, anglesRad, ax, ay)
% precomputes the distances between each control point when transformed
% under a rigid transformation (scaling/rotation). this is repeated for 
% each possible rotating defining the discretized edge directions;

T = [1/ax 0; 0 1/ay]; % scaling
N = length(xcp)*length(ycp); % total number of control pts 
numAngles = length(anglesRad);
E = zeros(N,N,numAngles); % E is a (N,N,numAngles) tensor
[xx,yy] = meshgrid(xcp, ycp); % xx and yy contain the N points
pts = zeros(numel(xx),2); % here we flatten meshgrid into vector 
pts(:,1) = reshape(xx,N,1);
pts(:,2) = reshape(yy,N,1);
% the number of unique pairs of points is : N^2 
[xxx,yyy] = meshgrid(pts(:,1), pts(:,2)); 
% del is a matrix defining the delta cartesian difference between the 
% pairs of points
del(1,:) = reshape(xxx - xxx', N*N,1);
del(2,:) = reshape(yyy - yyy', N*N,1);
% error checking 
maxVal = norm([min(xcp) min(ycp)]-[max(xcp) max(ycp)]);
maxVal = max([maxVal, ax*maxVal, ay*maxVal]);
% main loop
for i = 1:1:length(anglesRad)
    % M combines the scaling + rotation
    M = T*[cos(anglesRad(i)) sin(anglesRad(i)); -sin(anglesRad(i)) cos(anglesRad(i))];
    % multiple M by the direction vectors in del 
    vals = M*del;
    % store the exponential of all the scaled distances into E
    Etemp = reshape(exp(-vecnorm(vals)),[N N]);
    % make symmetric to avoid round-off error    
    E(:,:,i) = (Etemp + Etemp')/2; 
end
if ( max(max(E)) > maxVal )
   error('Error: max E value '); 
end

% for i = 1:1:length(anglesRad)
%     fprintf('Angle %i of %i\n',i,length(anglesRad));
%     M = T*[cos(anglesRad(i)) sin(anglesRad(i)); -sin(anglesRa(i)) cos(anglesRad(i))];    
%     l = 1;
%     for j = 1:1:length(xcp)       
%         for k = 1:1:length(ycp)
%             pt1 = [xcp(j) ycp(k)]';
%             p = 1;
%             Etemp = zeros(length(xcp),length(ycp));
%             parfor m = 1:1:length(xcp)       
%                 for n = 1:1:length(ycp)
%                     pt2 = [xcp(m) ycp(n)]';
%                     Etemp(l,p) = exp(norm(M*(pt2-pt1)));
%                     p = p + 1;
%                 end
%             end
%             E(:,:,i) = Etemp;
%             l = l + 1;
%             fprintf('Processed pt %i of %i\n',l,length(xcp)*length(ycp));
%         end
%     end    
% end
end