%function forecast = krig(X,Y,measurements,sig)
%
% Implements a linear kriging algorithm
%
% Derek Paley, 2015
function forecast = krig(X,Y,measurements,sig)
nargin
if ~nargin,
    rng(1)
    N = 300;
    M = 200;
    sig = 6;
    measurements = [round(2*rand(M,1)-1) N*rand(M,2)];
    [X,Y] = meshgrid(1:N);
else
    N = size(X,1);
    M = size(measurements,1);
end

forecast = zeros(N,N);
nmeas = size(measurements,1);
for ii=1:nmeas,
    dist = sqrt((X-measurements(ii,2)).^2+(Y-measurements(ii,3)).^2);
    weights = exp(-dist/sig)/nmeas;
    %weights(find(dist>3*sig)) = 0;
    forecast = forecast+weights*measurements(ii,1);
end

if ~nargin,
    figure(1), clf, colormap jet
    imagesc(1:N,1:N,forecast), hold on, axis xy
    ii = find(measurements(:,1)>0);
    jj = find(measurements(:,1)<0);
    plot(measurements(ii,2),measurements(ii,3),'wx')
    plot(measurements(jj,2),measurements(jj,3),'wo')
    axis image
    set(gca,'fontsize',16)
    colorbar
end

% tic
% dist = sqrt((X(:)*ones(1,M)-ones(N^2,1)*measurements(:,2)').^2+...
%     (Y(:)*ones(1,M)-ones(N^2,1)*measurements(:,3)').^2);
% weights = exp(-dist/sig);
% weights(find(dist>3*sig)) = 0;
% forecast2 = reshape(weights*measurements(:,1),N,N);
% toc
% 
% figure(2), clf, colormap jet
% imagesc(1:N,1:N,forecast2)
% axis image