clear; 
close all;
clc;

p1 = [1,1];
p2 = [3,4];

figure;
plot(p1(1),p1(2),'go'); hold on;
plot(p2(1),p2(2),'ro');
plot([p1(1) p2(1)],[p1(2) p2(2)],'b-');

th = atan2(p2(2)-p1(2),p2(1)-p1(1))+pi/2;
%th = pi/3;

ax = 1;
ay =0.1;
R1 = [cos(th) sin(th); -sin(th) cos(th)];
T1 = [1/ax 0; 0 1/ay];
D1 = T1*R1;
delp = p2-p1;
delpp = D1*delp';


%         ellipseParams.xc : x-center of ellipse
%         ellipseParams.yc : y-center of ellipse
%         ellipseParams.semiMinor : semi-minor axis of ellipse
%                                 : half the 
%         ellipseParams.semiMajor : semi-major axis of ellipse
%         ellipseParams.angle : right-handed angle of semi-major axis
%                               measured from x-axis
ellipseParams.mean = p1;
ellipseParams.semiMinor = ay;
ellipseParams.semiMajor = ax;
ellipseParams.angle = th;
numPts = 20;
[xe,ye] = ellipseCoords(ellipseParams, numPts)
plot(xe,ye,'k--');

axis equal;
norm(delp)
norm(delpp)