function standaloneWptCoordinator(runTime,cptRadius,N)


% parameters
% N = number of agents is given by the input argument
positionUpdateTimeout = 0.5; % in seconds
rosLoopRate = 4;
bundleUpdateIters = 10;

wptCoordStart = tic;
diaryFileName = ['WptCoordinatorLog_' datestr(now,'dd_mmm_yyyy_HHMMSS') '.txt'];
matFileName =  ['WptCoordinatorMat_' datestr(now,'dd_mmm_yyyy_HHMMSS') '.mat'];
diary(diaryFileName);
rosshutdown


bundle = cell(N,1);
bundleID = zeros(N,1); % store the ID of the bundle
bundleWptCounter = zeros(N,1);

agentLocation = zeros(N,2); % agentLocation records the location of agents, each row represents the [esat, north] location
agentAltitude = zeros(N,1);


captureRadius = cptRadius; % meter

rosinit;

r = robotics.Rate(rosLoopRate);
reset(r);

% use rosservice to get bundle
% bundleServer = rossvcserver('/bundle_server','bundle_manager/BUNDLE_REQUEST');
bundleClient = rossvcclient('/bundle_server');
bundleRequest = rosmessage(bundleClient);
disp('client done');

% setup the client to command quads
waypointClient = rossvcclient('command_waypoint');
waypointRequest = rosmessage(waypointClient);

% use rosmsg to get quad position
positionSub = rossubscriber('/MACE/UPDATE_POSITION','BufferSize', 10);% @positionCallback, 'BufferSize', 10);

while 1
    pause(1);
    % need to try to request bundle in while loop
    bundleRequest.VehicleID = uint8(1);
    try
        bundleResponse = call(bundleClient, bundleRequest, 'Timeout', 1);
    catch
        ; % do nothing
    end
    if exist('bundleResponse','var')
        if ~isempty(bundleResponse)
            if bundleResponse.BundleID > 0
                break;
            end
        end
    end
end

% ====== initialization done ========================

loopStart = tic;
iterationCounter = 1;
currentBundleID = zeros(N,1);

breakWhileLoop = 0;  % exit switch

while toc(loopStart) <= runTime+10
    % start main loop
    iterationStart = tic;
    if mod(iterationCounter,bundleUpdateIters) == 1
        % get bundle information every certain iterations
        for k = 1:N
            % request bundle from each agent
            bundleRequest.VehicleID = k;
            try
                bundleResponse = call(bundleClient, bundleRequest, 'Timeout', 5);
            catch
                % If the code catches an error at this moment, then it must be that
                % main program stops. We will issue an exit to the while
                % loop.
                breakWhileLoop = 1;
                break;
            end
            % update the bundle only if the bundle is new
            if bundleResponse.BundleID > currentBundleID(k)
                % TODO: change hardcode
                bundle{k} = [bundleResponse.Easting1 bundleResponse.Northing1;
                    bundleResponse.Easting2 bundleResponse.Northing2;
                    bundleResponse.Easting3 bundleResponse.Northing3;
                    bundleResponse.Easting4 bundleResponse.Northing4;
                    bundleResponse.Easting5 bundleResponse.Northing5]
                agentAltitude(k) = bundleResponse.Altitude1;
                bundleWptCounter(k) = 1;
                % update the currentBundleID
                currentBundleID(k) = bundleResponse.BundleID;
                
                % force to command the quad to a new waypoint
                waypointRequest.Timestamp = rostime('now');
                waypointRequest.VehicleID = k;   % Vehicle ID
                waypointRequest.CommandID = 3;
                [east, north] = F3toENU(bundle{k}(1,1), bundle{k}(1,2));
                waypointRequest.Easting = east; % Relative easting position to Datum
                waypointRequest.Northing = north; % Relative northing position to Datum
                waypointRequest.Altitude = agentAltitude(k);
                waypointResponse = call(waypointClient, waypointRequest, 'Timeout', 10);
                fprintf('Forcing 1st waypoint of new bundle now!\n');
                fprintf('Sending (%3.3f, %3.3f) to Quad %d\n', bundle{k}(1,1), bundle{k}(1,2),k);
                if ( waypointResponse.Success )
                    fprintf('VehicleID %d Wpt Command Sent.\n',k);
                else
                    fprintf('VehicleID %d Wpt Command Failed.\n',k);
                end
            end
        end
        fprintf('bundleRequested at %1.1f\n',toc(iterationStart));
    end
    
    if breakWhileLoop
        break;
    end
    
    agentUpdated = zeros(1,N);
    positionUpdateStart = tic;
    
    while (~all(agentUpdated))
        % when not all agents are updated OR tSample has not been reached, stay in the while loop
        % but if tSample has been passed, then grab one immediate position update and jump out of the while loop
        
        msg = positionSub.LatestMessage;
        
        % update the status in agentUpdated
        agentIndex =  msg.VehicleID ;
        agentUpdated(agentIndex) = 1;
        
        [xF3, yF3] = ENUtoF3(msg.Easting, msg.Northing);
        agentLocation(agentIndex,1) = xF3;
        agentLocation(agentIndex,2) = yF3;
        
        % if the time inside the while loop goes beyond
        % positionUpdateTimeout, then quit while loop
        if toc(positionUpdateStart) >= positionUpdateTimeout
            fprintf('Position update time out!\n Vehicle %d not updated.\n',find(agentUpdated==0));
            break;
        end
    end
    
    fprintf('Position update done at %1.1f\n',toc(iterationStart));
    
    % only examine the quads that have updated locations
    for k = 1:N
        %if agentUpdated(k) == 1
        distToWpt = norm(agentLocation(k,:)-bundle{k}(bundleWptCounter(k),:));
        fprintf('Distance of agent %d to wpt is %3.3f (capture radius = %3.3f) \n', k, distToWpt, captureRadius);
        if distToWpt <= captureRadius
            if bundleWptCounter(k) < 5 % TODO: fix this hardcode
                bundleWptCounter(k) = bundleWptCounter(k) + 1;
            end
            fprintf('Updating bundle wpt counter to %d\n',bundleWptCounter(k));
        end
        % packup a command packet and send it to quad
        waypointRequest.Timestamp = rostime('now');
        waypointRequest.VehicleID = k;   % Vehicle ID
        waypointRequest.CommandID = 3; % TODO: Set command ID enum in MACE
        fprintf('Sending (%3.3f, %3.3f) to Quad %d\n', bundle{k}(bundleWptCounter(k),1), bundle{k}(bundleWptCounter(k),2),k);
        [east, north] = F3toENU(bundle{k}(bundleWptCounter(k),1), bundle{k}(bundleWptCounter(k),2));
        wptCommand(k,1) = bundle{k}(bundleWptCounter(k),1);
        wptCommand(k,2) = bundle{k}(bundleWptCounter(k),2);
        waypointRequest.Easting = east; % Relative easting position to Datum
        waypointRequest.Northing = north; % Relative northing position to Datum
        waypointRequest.Altitude = agentAltitude(k);
        waypointResponse = call(waypointClient, waypointRequest, 'Timeout', 10);
        if ( waypointResponse.Success )
            fprintf('VehicleID %d Wpt Command Sent.\n',k);
        else
            fprintf('VehicleID %d Wpt Command Failed.\n',k);
        end
        fprintf('Command to quad %d done at %1.1f\n',k,toc(iterationStart));
        %end
        %         fprintf('Command done at %1.1f\n',toc(iterationStart)); %
        %         originally the above line is uncommented on June 18, 2019.
    end
    fprintf('Iteration time before waitfor(r): %1.1f\n',toc(iterationStart));
    waitfor(r);
    iterationCounter = iterationCounter + 1;
    fprintf('Iteration %d done using %1.1f\n',iterationCounter,toc(iterationStart));
    toc(wptCoordStart);
    
    % log data for
    wptCoordData.swarmState{iterationCounter} = agentLocation;
    wptCoordData.time(iterationCounter) = toc(wptCoordStart);
    wptCoordData.bundles{iterationCounter} = bundle;
    wptCoordData.wptCommand{iterationCounter} = wptCommand;
        
end
rosshutdown;
save(matFileName,'wptCoordData');
diary off;
quit;
end
