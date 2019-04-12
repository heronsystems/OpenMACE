clear; close all; clc;
addpath('../')
updatePath;

% R = 2;
% xmin = -57.5;
% xmax = 26.5;
% ymin = -11.5; 
% ymax = 11.5;

R = 20;
xmin = 0;
xmax = 400;
ymin = 0; 
ymax = 400;


[xpts1, ypts1, xinit1, yinit1] = lawnmowerRectangle(xmin,xmax, ymin, ymax, 1, R);
[xpts2, ypts2, xinit2, yinit2] = lawnmowerRectangle(xmin,xmax, ymin, ymax, 2, R);
[xpts3, ypts3, xinit3, yinit3] = lawnmowerRectangle(xmin,xmax, ymin, ymax, 3, R);
[xpts4, ypts4, xinit4, yinit4] = lawnmowerRectangle(xmin,xmax, ymin, ymax, 4, R);



figure;
plot([xmin xmin xmax xmax xmin],[ymin ymax ymax ymin ymin],'k-','linewidth',2);
hold on;
xlabel('X');
ylabel('Y');
plot(xpts1,ypts1,'ro-')
plot(xinit1,yinit1,'ro-')
plot(xpts2,ypts2,'bo-')
plot(xinit2,yinit2,'bo-')
plot(xpts3,ypts3,'mo-')
plot(xinit3,yinit3,'mo-')
plot(xpts4,ypts4,'co-')
plot(xinit4,yinit4,'co-')

for i = 1:1:floor( (ymax-ymin)/(2*R) / 2 )+1
   plot([xmin (xmax-xmin)/2+xmin],[ymin ymin] + i*2*R,'k--')
end


