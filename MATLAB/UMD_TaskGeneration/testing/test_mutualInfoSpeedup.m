clear; 
close all;
clc;

% create fake data
N = 100;
V = rand(N,N);
% randomly add some zeros and ones
M = floor(N^2/2);
zerosInd = randi(N,M,2);
onesInd = randi(N,M);
for m = 1:1:M
V(zerosInd(m,1), zerosInd(m,2)) = 0;
V(onesInd(m,1), onesInd(m,2)) = 1;
end
U = rand(N,N).*(1-V);
O = 1 - V - U;

figure; imagesc(V); colorbar;
figure; imagesc(U); colorbar;
figure; imagesc(O); colorbar;
figure; imagesc(V+U+O); colorbar;

% parameters
mZ = 3;
nZ = 100;
zval = linspace(-3,mZ+3,nZ);
for q = 1:1:nZ
    z_VU(q) = exp(-(zval(q))^2/2);
    z_O(q) = exp(-(zval(q)-mZ)^2/2);
end

nG = nZ; % number of discrete sensor levels
mG = 3; % sensitivity
gval = linspace(-3,mG+3,nG);
for l = 1:1:nG
    g_V(l) = exp(-(gval(l))^2/2);
    g_UO(l) = exp(-(gval(l)-mG)^2/2);
end
g_V = g_V ./ sum(g_V);
g_UO = g_UO ./ sum(g_UO);

%%

tic;
[H_C, I_C_GZ, totalEntropy] = mutualInformationMappingTarget_original(V, U, O, z_VU, z_O, g_V, g_UO);
toc;