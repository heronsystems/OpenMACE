function varargout = computeInformationGain(state,taskLocation,swarmModel,simParams,mutualInfoSurface,trueWorld)
%
% Description: compute the information collected along the planned path
%
% Input:
%   state : (1 x 4) vector, represents [x y xdot ydot] of the agent
%   taskLocation : (1 x 2) vector of target location
%   swarmModel : a struct to hold parameters
%   simParams: a struct to hold parameters
%
% Output:
%   collectedInformation: scalar value (used to be costAndPenalty)
%
%
% Sheng Cheng, 2018

% adopt to variable output size
nOutputs = nargout;
varargout = cell(1,nOutputs);

% initialization
varargout{1} = 0;





%% Euler method
% u = [];
% x = state';
% 
% path = x(1:2)';
% % first propogate the state to compute all future control actions and the
% % terminal state
% time = 0;

% for k = 1:round(simParams.T/simParams.dt)
%     u = [u waypointController(x,taskLocation(1),taskLocation(2),swarmModel.kp_wpt,swarmModel.kd_wpt,swarmModel.umax)];
%     x = x + simParams.dt*(swarmModel.A*x + swarmModel.B*u(:,end));
%     path = [path; x(1:2)'];
%     %effort = effort + simParams.dt*sqrt(u(1,end)*u(1,end)' + u(2,end)*u(2,end)');
%     time = time + simParams.dt;
%     % adaptive terminal time: if the agents is close to the taskLocation by
%     % Rsense, then we terminate the process of computing trajectory and
%     % effort
%     if sqrt((x(1)-taskLocation(1))^2+(x(2)-taskLocation(2))^2) <= swarmModel.Rsense/10
%         %disp('wpt reached in planner');
%         break;
%     end
% end

%% ode45 method
params.kp = swarmModel.kp_wpt;
params.kd = swarmModel.kd_wpt;
params.umax = swarmModel.umax;
params.N = 1; % this is for a single agent (even though ODE is for swarm)
params.A = swarmModel.A;
params.B = swarmModel.B;
params.xd = taskLocation(1);
params.yd = taskLocation(2);
params.Rsense = swarmModel.Rsense;
x0 = state';
delt = simParams.T; % max time 
odeOptions = odeset('Events', @(t,x)wptReachedEvent(t,x,params));
[timeHist, xHist] = ode45(@(t,x) swarmWptODE(t,x,params),[0 delt],x0, odeOptions);
%xsim(k,:)= xout(end,:);
%x0 = xsim(k,:);  
time = timeHist(end);
path = xHist(:,1:2);
x = xHist(end,:)';

%%
% collect the cells swiped by the sensor
reachedCell = zeros(size(mutualInfoSurface));

% calculate how many samples will be obtained along the path
nsamp = floor(time/simParams.Tsamp);
%path increment
for k = floor(linspace(1,length(path),nsamp))%
    % % the "|" operation is logical 'OR'.
    % given matrices A and B, or(A,B) returns array with logical 1 in an
    % index if either A or B have a nonzero entry at that index
    reachedCell = reachedCell | findCellsInView_LocalMatrixOutput(path(k,1), path(k,2),mutualInfoSurface,trueWorld.xcp,trueWorld.ycp,swarmModel.Rsense);
end



% collect all the information among the cells swiped by the sensor.
eta = 1;
varargout{1} = eta*max(max(reachedCell.*mutualInfoSurface))/time + (1-eta)*sum(sum(reachedCell.*mutualInfoSurface))/time;
%varargout{1} = sum(sum(reachedCell.*mutualInfoSurface))/time;

% if terminal state is required, return this vector
if nOutputs >= 2
    varargout{2} = x;
end

% if the resulting info surface is required, return the surface
if nOutputs >= 3
    varargout{3} = (1-reachedCell).*mutualInfoSurface;
end

if nOutputs >= 4
    varargout{4} = path;
end

%verification: make sure to put a breakpoint at line 72
% figure;subplot(1,2,1);
% imagesc(trueWorld.xcp,trueWorld.ycp,mutualInfoSurface);hold on;
% set(gca,'YDir','Normal')
% plot(path(1,1),path(1,2),'ro','linewidth',2)
% plot(path(:,1),path(:,2),'r-','linewidth',2);
% plot(taskLocation(1),taskLocation(2),'mo','linewidth',2);
% axis equal;
% axis tight;
% subplot(1,2,2);
% imagesc(trueWorld.xcp,trueWorld.ycp,reachedCell);
% hold on;
% numPts = 20;
% [xc, yc] = generateCircle(0, 0, swarmModel.Rsense, numPts);
% plot(path(1,1),path(1,2),'ro','linewidth',2)
% %plot(trueWorld.xcp(pathBinsX), trueWorld.ycp(pathBinsY),'k*');
% for i = 1:1:length(trueWorld.xcp)
%         plot(trueWorld.xcp(i)*ones(size(trueWorld.ycp)), trueWorld.ycp,'c.','linewidth',3);
% end
% plot(path(1,1)+xc,path(1,2)+yc,'r--','linewidth',1)
% plot(path(:,1),path(:,2),'r-','linewidth',2);
% plot(path(end,1)+xc,path(end,2)+yc,'r--','linewidth',1)
% plot(taskLocation(1),taskLocation(2),'mo','linewidth',2);
% set(gca,'YDir','Normal')
% axis equal;
% axis tight;
% pause;
% close all;
