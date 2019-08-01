%plotWptPerformance(swarmStateHist,swarmModel)
clear;
close all;
clc;
format compact;
%addpath('/home/wolek/Desktop/F3/Flight_Tests/02July2019/')
%load('/home/wolek/Desktop/F3/Flight_Tests/02July2019/F3FlightData_02_Jul_2019_153651.mat');
%load('/home/wolek/Desktop/F3/Flight_Tests/02July2019/F3FlightData_02_Jul_2019_154758.mat')
%load('WptCoordinatorMat_07_Jul_2019_195542.mat')
%load('F3FlightData_07_Jul_2019_195716.mat');
load('F3FlightData_19_Jul_2019_102003.mat');

%load('WptCoordinatorMat_08_Jul_2019_134650.mat');
%load('F3FlightData_08_Jul_2019_135121.mat');


close all;
rng default % For reproducibility

% flags
multiStart = 0;
opt1flag = 0; % double integrator with damping
opt2flag = 0; % double integrator with damping and delay
opt3flag = 0; % triple integrator with damping
opt4flag = 0; % triple integrator with damping and delay
wptCoordDataFlag = 1;
cutOffFlag = 1;
N = 2;
options = optimoptions('fmincon','Display','iter'); %,'MaxFunctionEvaluations',18,'MaxIterations',5);

if (wptCoordDataFlag)
    % vectorize
    t = wptCoordData.time(2:end) - wptCoordData.time(2);
    for j = 2:1:length(wptCoordData.time)
        xtemp = [];
        for k = 1:1:N
            xtemp = [xtemp; wptCoordData.swarmState{j}(:,k); 0; 0];
        end
        x(j-1,:) = xtemp';
        xd(j-1,:) = wptCoordData.wptCommand{j}(1,:);
        yd(j-1,:) = wptCoordData.wptCommand{j}(2,:);
    end
else
    % vectorize
    for j = 1:1:length(swarmStateHist)
        t(j) = swarmStateHist{j}.t;
        x(j,:) = swarmStateHist{j}.x;
        xd(j,:) = swarmStateHist{j}.xd;
        yd(j,:) = swarmStateHist{j}.yd;
    end
    
    for k = 1:1:swarmModel.N
        x(1,4*(k-1)+3)=0;
        x(1,4*(k-1)+4)=0;
    end
end

if (cutOffFlag)
    cutOffTime = 120;
    cutOffStep = interp1(t,1:1:length(t),cutOffTime,'nearest');
    t = t(1:cutOffStep);
    x = x(1:cutOffStep,:);
    xd = xd(1:cutOffStep,:);
    yd = yd(1:cutOffStep,:);
else
    cutOffStep = length(t);
end




%% double integrator with damping (default)
param_nom = [10 5 1 2]
d_nom = param_nom(4)/param_nom(3); % agent damping parameter
A_nom = [0 0 1 0; 0 0 0 1; 0 0 -d_nom 0; 0 0 0 -d_nom];
xsim_doubleint = simulateSys_doubleint(x(1,:)',t,xd,yd,param_nom(1),param_nom(2),param_nom(4),A_nom,swarmModel.B,swarmModel.N);
disp('Double Integrator default pOpt:');
param_nom
disp('Cost')
wptFollowCostFunc_doubleint(param_nom,t,x,xd,yd,swarmModel.B,swarmModel.N)

%% double integrator with damping (optimized)
% optimize free parameters are kp, kd, umax ,vmax
if ( opt1flag )
    lb = [1 1 0.25 0.25];
    ub = [20 20 3 3];
    p0 = [2.8156   13.8507    0.25    0.63];
    if ( multiStart )
        % multi-start method
        gs = GlobalSearch;
        problem = createOptimProblem('fmincon','x0',p0,'objective',@(p) wptFollowCostFunc_doubleint(p,t,x,xd,yd,swarmModel.B,swarmModel.N),'lb',lb,'ub',ub);
        gs.Display = 'iter';
        gs.NumStageOnePoints = 15;
        gs.NumTrialPoints = 30;
        gs.MaxTime = 10*60;
        [param_opt,fval,exitflag,output,solutions] = run(gs,problem);
    else
        [param_opt, cost_opt] = fmincon(@(p) wptFollowCostFunc_doubleint(p,t,x,xd,yd,swarmModel.B,swarmModel.N), p0, [], [], [], [], lb, ub, [], options );
    end
    disp('Double Integrator optimized pOpt:');
    param_opt
    disp('Cost')
    wptFollowCostFunc_doubleint(param_opt,t,x,xd,yd,swarmModel.B,swarmModel.N)
    % parse
    kp_opt = param_opt(1);
    kd_opt = param_opt(2);
    umax_opt = param_opt(3);
    vmax_opt = param_opt(4);
    d_opt = umax_opt/vmax_opt; % agent damping parameter
    A_opt = [0 0 1 0; 0 0 0 1; 0 0 -d_opt 0; 0 0 0 -d_opt];
    xsim_doubleint_opt = simulateSys_doubleint(x(1,:)',t,xd,yd,kp_opt,kd_opt,umax_opt,A_opt,swarmModel.B,swarmModel.N);
end

%% double integrator with damping and delay (optimized)
% optimize free parameters are kp, kd, umax, vmax, delay
if ( opt2flag )
    lb = [1 1 0.25 0.25 0];
    ub = [20 20 3 3 3];
    p0 = [2.8156   13.8507    0.3782    0.7167 1.25];
    
    if ( multiStart )
        % multi-start method
        clear gs problem;
        gs = GlobalSearch;
        problem = createOptimProblem('fmincon','x0',p0,'objective',@(p) wptFollowCostFunc_doubleint_delay(p,t,x,xd,yd,swarmModel.B,swarmModel.N),'lb',lb,'ub',ub);
        gs.Display = 'iter';
        gs.NumStageOnePoints = 15;
        gs.NumTrialPoints = 30;
        gs.MaxTime = 10*60;
        [param_opt,fval,exitflag,output,solutions] = run(gs,problem);
    else
        [param_opt, error_doubleint_opt] = fmincon(@(p) wptFollowCostFunc_doubleint_delay(p,t,x,xd,yd,swarmModel.B,swarmModel.N), p0, [], [], [], [], lb, ub, [], options );
    end
    % parse
    disp('Double Integrator with delay pOpt:');
    param_opt
    disp('Cost')
    wptFollowCostFunc_doubleint_delay(param_opt,t,x,xd,yd,swarmModel.B,swarmModel.N)
    kp_opt = param_opt(1);
    kd_opt = param_opt(2);
    umax_opt = param_opt(3);
    vmax_opt = param_opt(4);
    delay_opt = param_opt(5);
    d_opt = umax_opt/vmax_opt; % agent damping parameter
    A_opt = [0 0 1 0; 0 0 0 1; 0 0 -d_opt 0; 0 0 0 -d_opt];
    xsim_doubleint_delay_opt = simulateSys_doubleint_delay(x(1,:)',t,xd,yd,kp_opt,kd_opt,umax_opt,A_opt,swarmModel.B,swarmModel.N,delay_opt);
end
%% triple integrator with damping (optimized)
% optimize 5 free parameters are kp, kd, amax, vmax, jmax
if ( opt3flag )
    % vectorize
    for j = 1:1:cutOffStep
        xtemp = [];
        for k = 1:1:swarmModel.N
            xtemp = [xtemp; x(j,4*(k-1)+1:4*(k-1)+2)'; 0; 0; 0; 0];
        end
        xti(j,:) = xtemp;
    end
    
    lb = [1 1 0.5 0.25 0.25];
    ub = [20 20 3 3 3];
    p0 = [3.4   16.5    1.0    1.6    0.25];
    B_tripleint = [0 0; 0 0; 0 0; 0 0; 1 0; 0 1];
    % multi-start method
    if ( multiStart )
        clear gs problem;
        gs = GlobalSearch;
        problem = createOptimProblem('fmincon','x0',p0,'objective',@(p) wptFollowCostFunc_tripleint(p,t,xti,xd,yd,B_tripleint,swarmModel.N),'lb',lb,'ub',ub);
        gs.Display = 'iter';
        gs.NumStageOnePoints = 15;
        gs.NumTrialPoints = 30;
        gs.MaxTime = 10*60;
        [param_opt,fval,exitflag,output,solutions] = run(gs,problem);
    else
        % parse
        % param_opt = [10   10    0.5    2    2];
        % cost = wptFollowCostFunc_tripleint(param_opt,t,xti,xd,yd,B_tripleint,swarmModel.N)
        [param_opt, error_doubleint_opt] = fmincon(@(p) wptFollowCostFunc_tripleint(p,t,xti,xd,yd,B_tripleint,swarmModel.N), p0, [], [], [], [], lb, ub, [], options);
    end
    disp('Triple Integrator p0:');
    p0
    disp('Triple Integrator pOpt:');
    param_opt
    disp('Cost')
    wptFollowCostFunc_tripleint(param_opt,t,xti,xd,yd,B_tripleint,swarmModel.N)
    kp_opt = param_opt(1);
    kd_opt = param_opt(2);
    vmax_opt = param_opt(3);
    amax_opt = param_opt(4);
    jmax_opt = param_opt(5);
    dv_opt = amax_opt/vmax_opt; % agent damping parameter
    da_opt = jmax_opt/amax_opt;
    A_opt = [0 0 1 0 0 0;
        0 0 0 1 0 0;
        0 0 -dv_opt 0 1 0;
        0 0 0 -dv_opt 0 1;
        0 0 0 0 -da_opt 0;
        0 0 0 0 0 -da_opt];
    xsim_tripleint_opt = simulateSys_tripleint(xti(1,:)',t,xd,yd,kp_opt,kd_opt,jmax_opt,A_opt,B_tripleint,swarmModel.N);
end

%%
if ( opt4flag )
    % vectorize
    for j = 1:1:cutOffStep
        xtemp = [];
        for k = 1:1:swarmModel.N
            xtemp = [xtemp; x(j,4*(k-1)+1:4*(k-1)+2)'; 0; 0; 0; 0];
        end
        xti(j,:) = xtemp;
    end
    
    lb = [1 1 0.5 0.25 0.25 0];
    ub = [20 20 3 3 3 3];
    p0 = [4.4 15.0 0.72 1.3 0.75 1];
    B_tripleint = [0 0; 0 0; 0 0; 0 0; 1 0; 0 1];
    % multi-start method
    if ( multiStart )
        clear gs problem;
        gs = GlobalSearch;
        problem = createOptimProblem('fmincon','x0',p0,'objective',@(p) wptFollowCostFunc_tripleint_delay(p,t,xti,xd,yd,B_tripleint,swarmModel.N),'lb',lb,'ub',ub);
        gs.Display = 'iter';
        gs.NumStageOnePoints = 15;
        gs.NumTrialPoints = 30;
        gs.MaxTime = 10*60;
        [param_opt,fval,exitflag,output,solutions] = run(gs,problem);
    else
        % parse
        %param_opt = [10   10    0.5    2    2];
        %cost = wptFollowCostFunc_tripleint_delay(param_opt,t,xti,xd,yd,B_tripleint,swarmModel.N)
        [param_opt, error_doubleint_opt] = fmincon(@(p) wptFollowCostFunc_tripleint_delay(p,t,xti,xd,yd,B_tripleint,swarmModel.N), p0, [], [], [], [], lb, ub, [], options);
    end
    disp('Triple Integrator with delay p0:');
    p0
    disp('Triple Integrator with delay pOpt:');
    param_opt
    disp('Cost')
    wptFollowCostFunc_tripleint_delay(param_opt,t,xti,xd,yd,B_tripleint,swarmModel.N)
    kp_opt = param_opt(1);
    kd_opt = param_opt(2);
    vmax_opt = param_opt(3);
    amax_opt = param_opt(4);
    jmax_opt = param_opt(5);
    delay_opt = param_opt(6);
    dv_opt = amax_opt/vmax_opt; % agent damping parameter
    da_opt = jmax_opt/amax_opt;
    A_opt = [0 0 1 0 0 0;
        0 0 0 1 0 0;
        0 0 -dv_opt 0 1 0;
        0 0 0 -dv_opt 0 1;
        0 0 0 0 -da_opt 0;
        0 0 0 0 0 -da_opt];
    xsim_tripleint_delay_opt = simulateSys_tripleint_delay(xti(1,:)',t,xd,yd,kp_opt,kd_opt,jmax_opt,A_opt,B_tripleint,swarmModel.N, delay_opt);
end


for j = 1:1:swarmModel.N
    figure(j)
    subplot(2,1,1);
    plot(t,xd(:,j),'x');
    hold on;
    plot(t,x(:,(j-1)*4+1));
    plot(t,xsim_doubleint(:,(j-1)*4+1));
    if (opt1flag)
        plot(t,xsim_doubleint_opt(:,(j-1)*4+1));
    end
    if (opt2flag)
        plot(t,xsim_doubleint_delay_opt(:,(j-1)*4+1));
    end
    if (opt3flag)
        plot(t,xsim_tripleint_opt(:,(j-1)*6+1));
    end
    if (opt4flag)
        plot(t,xsim_tripleint_delay_opt(:,(j-1)*6+1));
    end
    
    set(gca,'FontSize',16)
    ylabel('X Pos (m)');
    subplot(2,1,2);
    plot(t,yd(:,j),'x');
    hold on;
    plot( t, x(:,(j-1)*4+2) );
    plot(t,xsim_doubleint(:,(j-1)*4+2));
    if (opt1flag)
        plot(t,xsim_doubleint_opt(:,(j-1)*4+2));
    end
    if (opt2flag)
        plot(t,xsim_doubleint_delay_opt(:,(j-1)*4+2));
    end
    if (opt3flag)
        plot(t,xsim_tripleint_opt(:,(j-1)*6+2));
    end
    if (opt4flag)
        plot(t,xsim_tripleint_delay_opt(:,(j-1)*6+2));
    end
    legend('Wpts','Actual','Default Double Int.','Double Int.','Double Int. w/Delay','Triple Int.','Triple Int. w/Delay');
    set(gca,'FontSize',16)
    xlabel('Time (sec.)')
    ylabel('Y Pos (m)');
    
    figure(j+10)
    hold on;
    
    diffX = xsim_doubleint(:,(j-1)*4+1) - x(:,(j-1)*4+1);
    diffY = xsim_doubleint(:,(j-1)*4+2) - x(:,(j-1)*4+2);
    r = sqrt(diffX.^2 + diffY.^2);
    plot(t,r);
    if (opt1flag)
        diffX = xsim_doubleint_opt(:,(j-1)*4+1) - x(:,(j-1)*4+1);
        diffY = xsim_doubleint_opt(:,(j-1)*4+2) - x(:,(j-1)*4+2);
        r = sqrt(diffX.^2 + diffY.^2);
        plot(t,r);
    end
    if (opt2flag)
        diffX = xsim_doubleint_delay_opt(:,(j-1)*4+1) - x(:,(j-1)*4+1);
        diffY = xsim_doubleint_delay_opt(:,(j-1)*4+2) - x(:,(j-1)*4+2);
        r = sqrt(diffX.^2 + diffY.^2);
        plot(t,r);
    end
    if (opt3flag)
        diffX = xsim_tripleint_opt(:,(j-1)*6+1) - x(:,(j-1)*4+1);
        diffY = xsim_tripleint_opt(:,(j-1)*6+2) - x(:,(j-1)*4+2);
        r = sqrt(diffX.^2 + diffY.^2);
        plot(t,r);
    end
    if (opt4flag)
        diffX = xsim_tripleint_delay_opt(:,(j-1)*6+1) - x(:,(j-1)*4+1);
        diffY = xsim_tripleint_delay_opt(:,(j-1)*6+2) - x(:,(j-1)*4+2);
        r = sqrt(diffX.^2 + diffY.^2);
        plot(t,r);
    end
    legend('Default Double Int.','Double Int.','Double Int. w/Delay','Triple Int.','Triple Int. w/Delay');
    set(gca,'FontSize',16)
    xlabel('Time (sec.)')
    ylabel('Distance to Actual Path (m)');    
end


%
% % end