
clear all; close all; clc;

N = [1:1:15];
LR_init = 2.2;
cl = 0.95;
LR_thresh = cl/(1-cl);

m = 1;
mu = m^2.*N/2 + log(LR_init);
sig = m^3;
F1 = 1-1/2*(1+erf( ( log(LR_thresh)-mu )./  (sig*sqrt(2) ) ));

m = 2;
mu = m^2.*N/2 + log(LR_init);
sig = m^3;
F2 = 1-1/2*(1+erf( ( log(LR_thresh)-mu )./  (sig*sqrt(2) ) ));

m = 3;
mu = m^2.*N/2 + log(LR_init);
sig = m^3;
F3= 1-1/2*(1+erf( ( log(LR_thresh)-mu )./  (sig*sqrt(2) ) ));

figure;
plot(N,F1,'ko-','linewidth',2);
hold on;
plot(N,F2,'ro-','linewidth',2);
plot(N,F3,'bo-','linewidth',2);
set(gca,'FontSize',16)
legend('m=1','m=2','m=3')
xlabel('Number of Views')
ylabel('Prob of Detection')
