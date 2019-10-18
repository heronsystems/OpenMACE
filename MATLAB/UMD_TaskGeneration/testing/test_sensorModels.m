clear;close all;
clc; format compact;

% Pd: 0.9500
% Pfa: 0.1000

m = 2;
nz = 25;

zi = linspace(-3,m+3,nz);
gi = exp(-zi.^2/2);
gi = gi ./ sum(gi); % normalize
hi= exp(-(zi-m).^2/2);
hi = hi ./ sum(hi); % normalize 

figure;
plot(zi,gi,'o-','linewidth',2);
hold on;
plot(zi,hi,'o-','linewidth',2);
xhand = xlabel('Grid Sensor Output, $g_l$');
yhand = ylabel('$p(g_l)$');
set(gca,'FontSize',16)
set(xhand,'Interpreter','Latex');
set(yhand,'Interpreter','Latex')
l = legend('$g_{l|V}$ (Noise)','$g_{l|U,O}$ (Signal)');
set(l,'Location','west')
set(l,'Interpreter','Latex')
grid on;
set(gcf,'Color','w')

figure;
plot(zi,gi,'ko-','linewidth',2);
hold on;
plot(zi,hi,'ro-','linewidth',2);
xhand = xlabel('Target Sensor Output, $z_q$');
yhand = ylabel('$p(z_q)$');
set(gca,'FontSize',16)
set(xhand,'Interpreter','Latex');
set(yhand,'Interpreter','Latex')
l = legend('$z_{q|V,U}$ (Noise)','$z_{q|O}$ (Signal)');
set(l,'Location','west')
set(l,'Interpreter','Latex')
grid on;
set(gcf,'Color','w')

%
disp('% Cell has low target likelihood')

p = 0;
q = 0.91;
r = 0.09;
fprintf('(p,q,r) = (%3.3f, %3.3f, %3.3f)\n',p,q,r);
computeInformationMetrics(m, p, q, r, nz)

%
disp('------------------------------------')
disp('% Cell has equal target likelihood')

p = 0;
q = 0.5;
r = 0.5;
fprintf('(p,q,r) = (%3.3f, %3.3f, %3.3f)\n',p,q,r);
computeInformationMetrics(m, p, q, r, nz)
%
disp('------------------------------------')
disp('% Cell has high target likelihood')

p = 0;
q = 0.09;
r = 0.91;
fprintf('(p,q,r) = (%3.3f, %3.3f, %3.3f)\n',p,q,r);
computeInformationMetrics(m, p, q, r, nz)

%
disp('------------------------------------')
disp('% Cell is obstacle')
p = 1;
q = 0;
r = 0;
fprintf('(p,q,r) = (%3.3f, %3.3f, %3.3f)\n',p,q,r);
computeInformationMetrics(m, p, q, r, nz)

%
disp('------------------------------------')
disp('% Cell unexplored and has uniform prior')
p = 0.33;
q = 0.33;
r = 0.33;
fprintf('(p,q,r) = (%3.3f, %3.3f, %3.3f)\n',p,q,r);
computeInformationMetrics(m, p, q, r, nz)

%
disp('------------------------------------')
disp('% Cell unexplored and has statistical prior')
p = 0.82;
q = 0.1746;
r = 0.0054;
fprintf('(p,q,r) = (%3.3f, %3.3f, %3.3f)\n',p,q,r);
computeInformationMetrics(m, p, q, r, nz)

