% testHeadingController.m
% 23-Aug-2018, A. Wolek
% Script to simulate double-integrator with damping and input saturation
% under a PID speed/heading control law or waypoint control

clear;
close all;
clc;

% assume this .m file is launching from ./testing folder
addpath('./')
cd ..
updatePath;


% user inputs
umax = 1.5; % max acceleration, m/s^2
vmax = 3; % max speed, m/s
x0 = [0 0 -vmax 0]; % initial condition [x y dxdt dydt]
T = 30; % simulation time
dt = 0.001; % euler's method time-step
skipFrames = 100; % controls speed of movie
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

for i = 2:1:N
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
        if norm(x(i-1,1:2) - [xd yd]) < R
            th = rand()*2*pi;
            xd = 10*cos(th);
            yd = 10*sin(th);
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
    xdot = A*x(i-1,:)' + B*u;
    x(i,:) = x(i-1,:) + xdot'*dt;
    t(i) = t(i-1) + dt;
end

% movie
if ( movieFlag )
    figure;
    pause;
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
        axis equal;
        xlim([-1 1]*boxWidth+x(i,1))
        ylim([-1 1]*boxWidth+x(i,2))
        set(gca,'FontSize',16)
        xlabel('X(m)')
        ylabel('Y(m)')
        drawnow;
        hold off;
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
