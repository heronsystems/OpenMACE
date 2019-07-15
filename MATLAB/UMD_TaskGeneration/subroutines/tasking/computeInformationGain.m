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

u = [];
effort = 0;
x = state';

switch swarmModel.utilityComputation
    case 'computeInformationGain'
        path = x(1:2)';
        % first propogate the state to compute all future control actions and the
        % terminal state
        time = 0;
        
        for k = 1:round(swarmModel.planningHorizon/simParams.dt)
            u = [u waypointController(x,taskLocation(1),taskLocation(2),swarmModel.kp_wpt,swarmModel.kd_wpt,swarmModel.umax)];
            x = x + simParams.dt*(swarmModel.A*x + swarmModel.B*u(:,end));
            path = [path; x(1:2)'];
            %effort = effort + simParams.dt*sqrt(u(1,end)*u(1,end)' + u(2,end)*u(2,end)');
            time = time + simParams.dt;
            % adaptive terminal time: if the agents is close to the taskLocation by
            % Rsense, then we terminate the process of computing trajectory and
            % effort
            if sqrt((x(1)-taskLocation(1))^2+(x(2)-taskLocation(2))^2) <= swarmModel.Rsense/25
                %disp('wpt reached in planner');
                break;
            end
        end
    case 'computeInformationGainLinearPath'
        effort = norm(taskLocation - x(1:2)); % total length of linear path
        ds = trueWorld.binWidth / 3;
        N = max(1,floor(effort / ds)); % number of pts to define path
        path = zeros(N,2);
        path(:,1) = linspace(x(1),taskLocation(1),N);
        path(:,2) = linspace(x(2),taskLocation(2),N);
        
end

% Artur : this was attempt to make computation faster by checking only unique
% bins but it turns out to be somewhat inaccurate due to rounding error may
% revisit this in future
% determine the unique bins that the path has gone through
% pathBinsX = max(round( (path(:,1) - trueWorld.xcp(1)) ./  trueWorld.binWidth ) + 1,1);
% pathBinsY = max(round( (path(:,2) - trueWorld.ycp(1)) ./  trueWorld.binWidth ) + 1,1);
% pathBinsX = min(pathBinsX, trueWorld.numBinsX);
% pathBinsY = min(pathBinsY, trueWorld.numBinsY);
% pathBins = [pathBinsX, pathBinsY];
% pathBins = unique(pathBins,'rows');

% % collect the cells swiped by the sensor
% reachedCell = zeros(size(mutualInfoSurface));
% for k = 1:size(pathBins,1)
%     % % the "|" operation is logical 'OR'.
%     % given matrices A and B, or(A,B) returns array with logical 1 in an
%     % index if either A or B have a nonzero entry at that index
%
%     %For Artur version (binned):
%     %reachedCell = reachedCell | findCellsInView_LocalMatrixOutpu(pathBins(k,1), pathBins(k,2),mutualInfoSurface,trueWorld.xcp,trueWorld.ycp,swarmModel.Rsense);
%
%     %For Sheng version (also binned):
%     %reachedCell = reachedCell | findCellsInView_LocalMatrixOutpu(cellCoordinates(k,:),mutualInfoSurface,trueWorld.xcp,trueWorld.ycp,swarmModel.Rsense/trueWorld.binWidth);
% end



% collect the cells swiped by the sensor
reachedCell = zeros(size(mutualInfoSurface));
% for k = 1:length(path)
%     % % the "|" operation is logical 'OR'.
%     % given matrices A and B, or(A,B) returns array with logical 1 in an
%     % index if either A or B have a nonzero entry at that index
%     reachedCell = reachedCell | findCellsInView_LocalMatrixOutput(path(k,1), path(k,2),mutualInfoSurface,trueWorld.xcp,trueWorld.ycp,swarmModel.Rsense);
% end


% calculate how many samples will be obtained along the path
nsamp = floor(time/simParams.Tsamp);
%path increment
kinc = floor(length(path)/nsamp);
for k = floor(linspace(1,length(path),nsamp))% for k = 1:kinc:(nsamp*kinc+1)
    % % the "|" operation is logical 'OR'.
    % given matrices A and B, or(A,B) returns array with logical 1 in an
    % index if either A or B have a nonzero entry at that index
    reachedCell = reachedCell | findCellsInView_LocalMatrixOutput(path(k,1), path(k,2),mutualInfoSurface,trueWorld.xcp,trueWorld.ycp,swarmModel.Rsense);
end



% collect all the information among the cells swiped by the sensor.
% NOTE: what is being computed below is the information per effort
% if norm(taskLocation - state(1:2))<swarmModel.Rsense
%     varargout{1} = 0; % if the task is within Rsense, then ignore this task
% else
    % info/effort
%     varargout{1} = sum(sum(reachedCell.*mutualInfoSurface))/effort;
    % info/arriving time
    
    varargout{1} = sum(sum(reachedCell.*mutualInfoSurface))/time;
    
% end

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


% check if the task is within sensing range, if so set its value equal to
% zero
% x = state';
% if sqrt((x(1)-taskLocation(1))^2+(x(2)-taskLocation(2))^2) <= swarmModel.Rsense/10
%     collectedInformation = 0;
% end


% collectedInformation = sum(sum(reachedCell.*mutualInfoSurface))/time;
% collectedInformation = sum(sum(reachedCell.*mutualInfoSurface));
% collectedInformation = sum(sum(reachedCell.*mutualInfoSurface))/swarmModel.planningHorizon;


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
