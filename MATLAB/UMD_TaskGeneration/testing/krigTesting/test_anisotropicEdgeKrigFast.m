clear; close all; clc;
rootPath = '/home/wolek/Desktop/Research/Projects/UMD/Heron/OpenMACE/MATLAB/UMD_TaskGeneration';
curPath = pwd;
addpath(rootPath);
addpath(curPath);
cd(rootPath)
updatePath;
cd(curPath)

load krigData;

%% analyze edges histogram
figure;
for i = 1:1:length(edgeDir)
   if (edgeDir(i) < 0)
      edgeDir(i) = pi+edgeDir(i); 
   end
end

%%
inc = 3;
nbins = floor(180/inc);
histogram(edgeDir*180/pi,nbins)
binEdges = [0:inc:180];
npeaks = 4;
theta = diff(binEdges)/2+binEdges(1:end-1);
sep = inc;
[N,~] = histcounts(edgeDir*180/pi,binEdges);
[peaks,weights] = findPeaks(N, npeaks);
peakVal = N(peaks);
if (min(weights) == 0)
   error('Error with weight computation'); 
end
%
hold on;
plot(theta(peaks),peakVal,'k*','linewidth',2,'MarkerSize',5);  
xlim([0 180]);

%%
%tic
%disp('Precomputing transformed distances and loading into memory...');
%E = precomputeReducedDistances(xcp, ycp, theta, ax, ay);
% % save('E.mat','-v7.3','E')
% load E.mat
%toc
[xx,yy] = meshgrid(xcp,ycp);


tic
%forecast = anisotropicAdaptiveKrig(xx,yy,measurements,ax,ay, peaks, weights);
%forecast = anisotropicAdaptiveKrigFast(xx,yy,measurements, E, peaks, weights);
forecast = anisotropicAdaptiveKrigFast2(xx,yy,measurements, ax,ay, peaks, weights, xcp, ycp);

toc


set(gca,'FontSize',16)
xlabel('Angle of Edges (deg.)')
ylabel('Number of Edges')

for i = 1:1:npeaks
    text(peaks(i),peakVal(i)+max(weights)*0.01,num2str(weights(i)));  
end


%% adaptive anisotropic krig

figure;
imagesc(forecast,'XData', xcp, 'YData', ycp); hold on;
set(gca,'YDir','Normal');
for i = 1:1:numSamples
    plot(xr(i) + xc, yr(i) + yc, 'r-'); hold on;
end
axis equal;
title('Adaptive (4-Peak) Anisotropic Kriging')
set(gca,'FontSize',16)
xlabel('X')
ylabel('Y')
xlim([trueWorld.minX trueWorld.maxX]);
ylim([trueWorld.minY trueWorld.maxY]);
colorbar;

 