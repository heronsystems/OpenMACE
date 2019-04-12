% test_minEffortControl.m
% 09-Dec-2018, S. Cheng
% Script to simulate double-integrator with damping and input saturation
% under an minimum control effort controller, with comparison to a PID
% controller.

clear;
close all;
clc;

% assume this .m file is launching from ./testing folder
% addpath('./')
% cd ..
% updatePath;


% user inputs
umax = 1.5; % max acceleration, m/s^2
vmax = 3; % max speed, m/s
x0 = [0 0 -vmax 0]; % initial condition [x y dxdt dydt]
T = 30; % simulation time
dt = 0.1; % euler's method time-step
skipFrames = 1; % controls speed of movie
L = 5; % length of heading indicator
boxWidth = 15; % width of movie window

movieFlag = 1; % flag to turn movie on/off
plotFlag = 0;

% heading/speed control
hsControlFlag = 0; % set wptControlFlag = 0 if setting hsControlFlag = 0
Vd = 2; % initial desired speed
psid = mod(-45*pi/180, 2*pi); % initial desired course
Tchange = 5; % sec, how often speed/heading command changes
kp_hs = 50.0; % control gains
kd_hs = 0.00;
ki_hs = 0.00;

% waypoint control
wptControlFlag = 1; % set hsControlFlag = 0 if setting wptControlFlag = 1
xd = 5; % initial waypoint x
yd = 10; % initial waypoint y

wptCost = [];
wptTime = [];
singleWptCost = 0;
Rcost = eye(2);

R = 1; % capture radus
kp_wpt = 10.0;
kd_wpt = 5.0;

% dynamics
d = umax/vmax;
A = [0 0 1 0;
    0 0 0 1;
    0 0 -d 0;
    0 0 0 -d];
B = [0 0;
    0 0;
    1 0;
    0 1];

% initial setpoints
if ( hsControlFlag )
    xdotd = Vd*cos(psid); % desired speeds
    ydotd = Vd*sin(psid);
end
if ( wptControlFlag )
    th = linspace(0,2*pi,20); % for plotting
    xc = R*cos(th);
    yc = R*sin(th);
end

% initialize simulation
N = floor(T/dt);
Nchange = floor(Tchange/dt);
x(1,:) = x0';
uhist(1,:) = [0 0];
t(1) = 0;
Vdhist(1) = Vd;
psidhist(1) = psid;

% initialize error variables
if ( hsControlFlag )
    ex = struct;
    ex.last = 0;
    ex.sum = 0;
    
    ey = struct;
    ey.last = 0;
    ey.sum = 0;    
end

destinationCount = 1;

i = 1;
while(1)
    i = i+1;
    % update setpoint and/or compute control
    if ( hsControlFlag )
        if mod(i,Nchange)==0
            psid = mod(rand()*2*pi,2*pi);
            Vd = rand()*(vmax-0.5)+0.5;
            xdotd = Vd*cos(psid);
            ydotd = Vd*sin(psid);
        end
        errorx = x(i-1,3) - xdotd;
        errory = x(i-1,4) - ydotd; 
        ex.sum = ex.sum + errorx*dt;
        ey.sum = ey.sum + errory*dt;
        % control law
        u(1,1) = -kp_hs*(errorx)-kd_hs*( (errorx-ex.last)/dt )-ki_hs*ex.sum;
        u(2,1) = -kp_hs*(errory)-kd_hs*( (errory-ey.last)/dt )-ki_hs*ey.sum;        
        % update 
        ex.last = errorx;
        ey.last = errory;
        % store
        Vdhist(i) = Vd;
        psidhist(i) = psid;
    end
    if ( wptControlFlag )
        if destinationCount < 5 && norm(x(i-1,1:2) - [xd yd]) < R
                th = rand()*2*pi;
                xd = 10*cos(th);
                yd = 10*sin(th);
                destinationCount = destinationCount + 1;
                wptCost = [wptCost singleWptCost];
                wptTime = [wptTime i];
                singleWptCost = 0;
                
        elseif destinationCount == 5 && norm(x(i-1,1:2) - [xd yd]) >= R
            N = i;%i+1
        elseif destinationCount == 5 && norm(x(i-1,1:2) - [xd yd]) < R
            wptCost = [wptCost singleWptCost];
            wptTime = [wptTime i];
            wptTime(2:end) = wptTime(2:end)-wptTime(1:end-1);
            break;
        end
        u(1,1) = -kp_wpt*(x(i-1,1) - xd) - kd_wpt*x(i-1,3);
        u(2,1) = -kp_wpt*(x(i-1,2) - yd) - kd_wpt*x(i-1,4);
        xdhist(i) = xd;
        ydhist(i) = yd;
    end
    % saturate
    if ( norm(u) > umax )
        u = u/norm(u)*umax;
    end
    uhist(i,:) = u';
    
    singleWptCost = singleWptCost + u'*Rcost*u*dt;
    
    xdot = A*x(i-1,:)' + B*u;
    x(i,:) = x(i-1,:) + xdot'*dt;
    t(i) = t(i-1) + dt;
end


temp = expm(dt*[A B;zeros(2,6)]);
Ad = temp(1:4,1:4);
Bd = temp(1:4,5:6);

runtime = [];
minCost = [];
minEffortTime = [];

xMinEffort = x0';
xdMinEffort = [];
ydMinEffort = [];
targetLocation = unique([xdhist;ydhist]','rows','stable')';
targetLocation(:,1) = [];


for j = 1:size(targetLocation,2)
% Rcost = eye(2);
tic;
[t_f,controls,singleMinCost] = minControlEffort(Ad,Bd,dt,xMinEffort(:,end),targetLocation(:,j),umax,Rcost,1);
minCost = [minCost singleMinCost];
minEffortTime = [minEffortTime t_f];

runtime = [runtime toc];

for k = 1:t_f-1
    xMinEffort = [xMinEffort Ad*xMinEffort(:,end)+Bd*controls(2*k-1:2*k)];
    xdMinEffort = [xdMinEffort targetLocation(1,j)];
    ydMinEffort = [ydMinEffort targetLocation(2,j)];
end

end
% plot(x(1,:),x(2,:));
% hold on;
% plot(x(1,1),x(2,1),'ro');
% plot(x(1,end),x(2,end),'b*');
% axis equal

% movie
if ( movieFlag )
    figure;
    pause;
    if N < size(xMinEffort,2)-1
        for i = 1:skipFrames:N
            plot(x(1:i,1), x(1:i,2),'k-','linewidth',2);
            hold on;
            plot(x(i,1), x(i,2),'ko','linewidth',2);
            grid on;
            if ( wptControlFlag )
                plot(xdhist(1:i) ,ydhist(1:i) ,'ro','MarkerSize',10,'linewidth',1);
                grid on;
                hold on;
                plot(xdhist(i) +xc,ydhist(i) +yc,'r--')
            end
            if ( hsControlFlag )
                arrowX = [x(i,1) x(i,1)+L*cos(psidhist(i))];
                arrowY = [x(i,2) x(i,2)+L*sin(psidhist(i))];
                plot(arrowX,arrowY,'m--');
            end
            
            plot(xMinEffort(1,1:i), xMinEffort(2,1:i),'b-','linewidth',2);
            hold on;
            plot(xMinEffort(1,i), xMinEffort(2,i),'bo','linewidth',2);
            grid on;
            if ( wptControlFlag )
                plot(xdMinEffort(1:i) ,ydMinEffort(1:i) ,'go','MarkerSize',10,'linewidth',1);
                grid on;
                hold on;
                plot(xdMinEffort(i) +xc,ydMinEffort(i) +yc,'g--')
            end
            
            axis equal;
            xlim([min(x(i,1),xMinEffort(1,i))-boxWidth, max(x(i,1),xMinEffort(1,i))+boxWidth]);
            ylim([min(x(i,2),xMinEffort(2,i))-boxWidth, max(x(i,2),xMinEffort(2,i))+boxWidth]);
            set(gca,'FontSize',16)
            xlabel('X(m)')
            ylabel('Y(m)')
            drawnow;
            hold off;
        end
        for i = N+1:skipFrames:size(xMinEffort,2)-1
            
            plot(x(1:N,1), x(1:N,2),'k-','linewidth',2);
            hold on;
            grid on;
            plot(xdhist(1:N) ,ydhist(1:N) ,'ro','MarkerSize',10,'linewidth',1);
            
            plot(xMinEffort(1,1:i), xMinEffort(2,1:i),'b-','linewidth',2);
            hold on;
            plot(xMinEffort(1,i), xMinEffort(2,i),'bo','linewidth',2);
            grid on;
            if ( wptControlFlag )
                plot(xdMinEffort(N:i) ,ydMinEffort(N:i) ,'go','MarkerSize',10,'linewidth',1);
                grid on;
                hold on;
                plot(xdMinEffort(i) +xc,ydMinEffort(i) +yc,'g--')
            end
            
            axis equal;
            xlim([min(x(N,1),xMinEffort(1,i))-boxWidth, max(x(N,1),xMinEffort(1,i))+boxWidth]);
            ylim([min(x(N,2),xMinEffort(2,i))-boxWidth, max(x(N,2),xMinEffort(2,i))+boxWidth]);
            set(gca,'FontSize',16)
            xlabel('X(m)')
            ylabel('Y(m)')
            drawnow;
            hold off;
        end
    else
        for i = 1:skipFrames:size(xMinEffort,2)-1
            plot(x(1:i,1), x(1:i,2),'k-','linewidth',2);
            hold on;
            plot(x(i,1), x(i,2),'ko','linewidth',2);
            grid on;
            if ( wptControlFlag )
                plot(xdhist(1:i) ,ydhist(1:i) ,'ro','MarkerSize',10,'linewidth',1);
                grid on;
                hold on;
                plot(xdhist(i) +xc,ydhist(i) +yc,'r--')
            end
            if ( hsControlFlag )
                arrowX = [x(i,1) x(i,1)+L*cos(psidhist(i))];
                arrowY = [x(i,2) x(i,2)+L*sin(psidhist(i))];
                plot(arrowX,arrowY,'m--');
            end
            
            plot(xMinEffort(1,1:i), xMinEffort(2,1:i),'b-','linewidth',2);
            hold on;
            plot(xMinEffort(1,i), xMinEffort(2,i),'bo','linewidth',2);
            grid on;
            if ( wptControlFlag )
                plot(xdMinEffort(1:i) ,ydMinEffort(1:i) ,'go','MarkerSize',10,'linewidth',1);
                grid on;
                hold on;
                plot(xdMinEffort(i) +xc,ydMinEffort(i) +yc,'g--')
            end
            
            axis equal;
            xlim([min(x(i,1),xMinEffort(1,i))-boxWidth, max(x(i,1),xMinEffort(1,i))+boxWidth]);
            ylim([min(x(i,2),xMinEffort(2,i))-boxWidth, max(x(i,2),xMinEffort(2,i))+boxWidth]);
            set(gca,'FontSize',16)
            xlabel('X(m)')
            ylabel('Y(m)')
            drawnow;
            hold off;
        end
        kk = size(xMinEffort,2)-1;
        for i = kk+1:skipFrames:N
            
            plot(xMinEffort(1,1:kk), xMinEffort(2,1:kk),'b-','linewidth',2);
            hold on;
            grid on;
            if ( wptControlFlag )
                plot(xdMinEffort(1:kk) ,ydMinEffort(1:kk) ,'go','MarkerSize',10,'linewidth',1);
                grid on;
                hold on;
                plot(xdMinEffort(kk) +xc,ydMinEffort(kk) +yc,'g--')
            end
            
            plot(x(1:i,1), x(1:i,2),'k-','linewidth',2);
            hold on;
            plot(x(i,1), x(i,2),'ko','linewidth',2);
            grid on;
            if ( wptControlFlag )
                plot(xdhist(1:i) ,ydhist(1:i) ,'ro','MarkerSize',10,'linewidth',1);
                grid on;
                hold on;
                plot(xdhist(i) +xc,ydhist(i) +yc,'r--')
            end
            if ( hsControlFlag )
                arrowX = [x(i,1) x(i,1)+L*cos(psidhist(i))];
                arrowY = [x(i,2) x(i,2)+L*sin(psidhist(i))];
                plot(arrowX,arrowY,'m--');
            end
            
            axis equal;
            xlim([min(x(i,1),xMinEffort(1,kk))-boxWidth, max(x(i,1),xMinEffort(1,kk))+boxWidth]);
            ylim([min(x(i,2),xMinEffort(2,kk))-boxWidth, max(x(i,2),xMinEffort(2,kk))+boxWidth]);
            set(gca,'FontSize',16)
            xlabel('X(m)')
            ylabel('Y(m)')
            drawnow;
            hold off;
        end
    end
end

if ( plotFlag )
    if ( hsControlFlag )
        
        figure;
        subplot(3,2,1)
        plot(t,Vdhist,'r','linewidth',2)
        hold on;
        Vhist = sqrt(x(:,3).^2 + x(:,4).^2);
        plot(t,Vhist,'b','linewidth',2)
        grid on;
        ylabel('Speed (m/s)')
        legend('Command','Value','Location','northeast')
        set(gca,'FontSize',16)
        subplot(3,2,3)
        plot(t,psidhist*180/pi,'r','linewidth',2)
        hold on;
        psihist = mod(atan2(x(:,4),x(:,3)),2*pi);
        plot(t,psihist*180/pi,'b','linewidth',2)
        grid on;
        ylabel('Course (deg)')
        set(gca,'FontSize',16)
        subplot(3,2,5)
        plot(t,uhist(:,1),'r-','linewidth',2)       
        hold on;
        plot(t,uhist(:,2),'b-','MarkerSize',10,'linewidth',2);
        set(gca,'FontSize',16)
        grid on;
        xlabel('Time (sec)')
        ylabel('Acc. (m/s^2)')
        legend('a_x','a_y','Location','southeast')       
        subplot(3,2,[2 4 6])
        plot(x(:,1), x(:,2),'k-','linewidth',2);
        axis equal;
        grid on;
        set(gca,'FontSize',16)
        xlabel('X(m)')
        ylabel('Y(m)')
        
        figure;
        umag = sqrt(uhist(:,1).^2 + uhist(:,2).^2);
        plot(t,umag,'k','linewidth',2)
        hold on;
        grid on;
        xlabel('Time (sec)')
        ylabel('|u| (m/s^2)')
        set(gca,'FontSize',16)

        
        % plot
        figure;
        subplot(2,1,1)
        xdotdhist = Vdhist.*cos(psidhist);
        ydotdhist = Vdhist.*sin(psidhist);
        plot(t,xdotdhist,'r--','linewidth',2)
        hold on;
        plot(t,ydotdhist,'m-','linewidth',2)
        plot(t,x(:,3),'b','linewidth',2)
        plot(t,x(:,4),'k','linewidth',2)
        grid on;
        ylabel('Speed (m/s)')
        legend('vx','vy')
        set(gca,'FontSize',16)
        subplot(2,1,2)
        plot(t,uhist(:,1),'r-','linewidth',2)       
        hold on;
        plot(t,uhist(:,2),'b-','MarkerSize',10,'linewidth',2);
        set(gca,'FontSize',16)
        grid on;
        xlabel('Time (sec)')
        ylabel('Acc. (m/s^2)')
        legend('a_x','a_y','Location','eastoutside')
        set(gca,'FontSize',16)
    end
    if (wptControlFlag)
        figure;
        subplot(2,1,1)
        plot(t,xdhist,'r--','linewidth',2);
        hold on;
        plot(t,ydhist,'b--','linewidth',2);
        plot(t,x(:,1),'m','linewidth',2)
        plot(t,x(:,2),'k','linewidth',2)
        legend('x_d','y_d','x','y','Location','eastoutside')
        grid on;
        set(gca,'FontSize',16)
        ylabel('Position (m)')
        
        subplot(2,1,2)
        plot(t,uhist(:,1),'r-','MarkerSize',10,'linewidth',2);
        hold on;
        plot(t,uhist(:,2),'b-','MarkerSize',10,'linewidth',2);
        set(gca,'FontSize',16)
        grid on;
        xlabel('Time (sec)')
        ylabel('Acc. (m/s^2)')
        legend('a_x','a_y','Location','eastoutside')
        set(gca,'FontSize',16)
        
        
        figure;
        plot(xdhist,ydhist,'ro-','MarkerSize',10,'linewidth',1);
        grid on;
        hold on;
        plot(x(:,1), x(:,2),'k-','linewidth',1);
        axis equal;
        xlabel('X(m)')
        ylabel('Y(m)')
        
    end
    
end
