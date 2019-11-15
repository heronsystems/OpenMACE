% test script for trajectory following with velocity commands.'
% clear; close all;clc

% consider a single integrator dynamical system
% integrate vx, vy to obtain px and py

% desired trajectory is described by tpx, tpy (and tvx, tvy)

commandingTime = 1; % send the command every 1 second

% ============ Case 1: straight line =========================
% % consider a straight line so that the desired trajectory is described by
% % a * tpx + b * tpy + c = 0
% a = -1;
% b = 1;
% c = 0;
% 
% % desired velocity is 1 
v = @(t) 1;
% % (can be made to a sinusoidal function to add complexity)
% v = @(t) sin(2*pi/100*t);
% 
% initial condition
px = 5;
py = 4;
% 
% % determine the minimum distance spot on the trajectory
% % use the tangent velocity as the forward velocity
% % normal velocity should be the distance to the spot divided by the commandingTime
% 
% for k = 1:50
%     % x0 and y0 are the closed point to the vehicle current location on the desired trajectory
%     x0 = (b*(b*px(end)-a*py(end))-a*c)/(a^2 + b^2);
%     y0 = (a*(-b*px(end)+a*py(end))-b*c)/(a^2 + b^2);
%     
%     % dist is the minimum distance
%     dist = abs(a*px(end)+b*px(end)+c)/sqrt(a^2 + b^2);
%     
%     v_tan = [1 1]/norm([1 1])*v(k*commandingTime);
%     v_norm = [x0-px(end) y0-py(end)]/commandingTime;
%     
%     px = [px px(end) + commandingTime*(v_tan(1)+v_norm(1))+0.01*randn(1)];
%     py = [py py(end) + commandingTime*(v_tan(2)+v_norm(2))+0.01*randn(1)];
% end
% 
% plot(px,py);axis equal;grid on

% ================ Case 2: cubic bspline ====================

% anchor points
% tpx = [5 8 5 0 -5 -8 -5 0];
% tpy = [5 0 -5 0 5 0 -5 0];

% circular trajectory
tpx = [5 8 5 -2 -10 -13 -10 -2 5];
tpy = [5 0 -5  -6 -5 0 5  6 5];

% snake trajectory
% tpx = [12 9 6 3 0 -3 -6 -9 -12];
% tpy = [-3 -6 -3 -6 -3 -6 -3 -6 -3];

% specify initial and final slope
% slope0 = 0;
% slopeF = -1;

% subplot(1,2,1);
% plot(tpx,tpy,tpx,tpy,'o');axis equal;

% anchor times
t = linspace(0,60,length(tpx));
% query times
t_query = 0:1:max(t);

x_query = spline(t,[tpx],t_query);
y_query = spline(t,[tpy],t_query);

x_coef = spline(t,[tpx]);
y_coef = spline(t,[tpy]);

figure;
% subplot(1,2,2);
plot(tpx,tpy,'s',x_query,y_query,':.');axis equal;hold on;
axis([-9,9,-7,7]);

% compute the closet point on the spline, and its distance and fractional
% arc length
[xy,distance,t_a] = distance2curve([tpx;tpy]',[8,5],'spline');

v_tan_stor = [];
v_norm_stor = [];

for k = 1:100
    % x0 and y0 are the closed point to the vehicle current location on the desired trajectory
    [xy,distance,t_a,coef,t_b] = distance2curve([tpx;tpy]',[px(end),py(end)],'spline');
    x0 = xy(1);
    y0 = xy(2);
    
    % dist is the minimum distance
    dist = distance;
    
    vx = 0;
    vy = 0;
    
    for j = 1:size(coef,2)-1
        vx = vx + coef(1,end-j)*j*(t_a-t_b)^j;
        vy = vy + coef(2,end-j)*j*(t_a-t_b)^j;
    end
    
    ax = 0;
    ay = 0;
    for j = 1:size(coef,2)-1
        ax = ax + coef(1,end-j)*j*(j-1)*(t_a-t_b)^(j-1);
        ay = ay + coef(2,end-j)*j*(j-1)*(t_a-t_b)^(j-1);
    end
    
    v_tan = [vx vy]/norm([vx vy])*v(k*commandingTime);
    v_norm = 1*[x0-px(end) y0-py(end)]/commandingTime;% + 0.4*[ax ay]/norm([ax ay]);
    
    v_tan_stor = [v_tan_stor;v_tan];
    v_norm_stor = [v_norm_stor;v_norm];
    
    px = [px px(end) + commandingTime*(v_tan(1)+v_norm(1))+0.01*randn(1)];
    py = [py py(end) + commandingTime*(v_tan(2)+v_norm(2))+0.01*randn(1)];
    
    h_agent = plot(px(end),py(end),'o');
    h_agent_vtan = quiver(px(end),py(end),v_tan(1),v_tan(2),'Color','r');
    h_agent_vnorm = quiver(px(end),py(end),v_norm(1),v_norm(2),'Color','b');
    
    drawnow;
    
    pause(0.05);
    set(h_agent,'visible','off');
    set(h_agent_vtan,'visible','off');
    set(h_agent_vnorm,'visible','off');
end

% hold on;
plot(px,py);axis equal;grid on

% figure;
% plot(v_tan_stor);
