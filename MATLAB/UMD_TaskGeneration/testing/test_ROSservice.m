rosshutdown
% IP = '127.0.0.1';
% setenv('ROS_MASTER_URI',['http://' IP ':11311']) % ROS Core location
% setenv('ROS_IP',IP) % MATLAB location

% bundleSource shouldn't show up in the standalone waypoint coordinator
% the following is for test only
global bundleSource
bundleSource = [1 10 2 0 4 0 6 0 8 0 10 0;
                1 5  0 2 0 4 0 6 0 8 0 10];

while isempty(bundleSource)
    % wait until bundleSource is updated in the main program
end
N = size(bundleSource,1);

bundle = cell(N,1);
bundleID = zeros(N,1); % store the ID of the bundle
bundleWptCounter = zeros(N,1);

agentLocation = zeros(N,2); % agentLocation records the location of agents, each row represents the [esat, north] location
agentAltitude = zeros(N,1);

positionUpdateTimeout = 0.5; % in seconds
captureRadius = 2; % meter

% roscore;
rosinit;

r = robotics.Rate(4);
reset(r);

% use rosservice to get bundle
bundleServer = rossvcserver('/bundle_server','bundle_manager/BUNDLE_REQUEST',@test_ROSserver);
disp('server done')
bundleClient = rossvcclient('/bundle_server');
bundleRequest = rosmessage(bundleClient);
disp('client done')

% send datum
datumClient = rossvcclient('command_datum');
datumRequest = rosmessage(datumClient);
datumRequest.CommandID = 0; % TODO: Set command ID enum in MACE
datumRequest.LatitudeDeg = 38.973699;
datumRequest.LongitudeDeg = -76.921897;
datumRequest.Timestamp = rostime('now');
datumRequest.VehicleID = 1; % Not necessary for this
datumResponse = call(datumClient, datumRequest, 'Timeout', 5);

% setup the client to command quads
waypointClient = rossvcclient('command_waypoint');
waypointRequest = rosmessage(waypointClient);

% use rosmsg to get quad position
positionSub = rossubscriber('/MACE/UPDATE_POSITION','BufferSize', 10);% @positionCallback, 'BufferSize', 10);

pause(2);

% ====== initialization done ========================

loopStart = tic;
iterationCounter = 1;
currentBundleID = zeros(N,1);

while toc(loopStart) <= 30
    % start main loop
    iterationStart = tic;
    if mod(iterationCounter,10) == 1
        % get bundle information every certain iterations
        for k = 1:N
            bundleRequest.VehicleID = k;
            bundleResponse = call(bundleClient, bundleRequest, 'Timeout', 5);
            % update the bundle only if the bundle is new
            if bundleResponse.BundleID > currentBundleID(k)
                bundle{k} = [bundleResponse.Easting1 bundleResponse.Northing1;
                    bundleResponse.Easting2 bundleResponse.Northing2;
                    bundleResponse.Easting3 bundleResponse.Northing3;
                    bundleResponse.Easting4 bundleResponse.Northing4;
                    bundleResponse.Easting5 bundleResponse.Northing5];
                agentAltitude(k) = bundleResponse.Altitude1;
                bundleWptCounter(k) = 1;
                % update the currentBundleID
                currentBundleID(k) = bundleResponse.BundleID;
                
                % force to command the quad to a new waypoint
                waypointRequest.Timestamp = rostime('now');
                waypointRequest.VehicleID = k;   % Vehicle ID
                waypointRequest.CommandID = 3; % TODO: Set command ID enum in MACE
                [east, north] = F3toENU(bundle{k}(1,1), bundle{k}(1,2));
                waypointRequest.Easting = east; % Relative easting position to Datum
                waypointRequest.Northing = north; % Relative northing position to Datum
                waypointRequest.Altitude = agentAltitude(k);
                waypointResponse = call(waypointClient, waypointRequest, 'Timeout', 10);
            end
        end
        fprintf('bundleRequested at %1.1f\n',toc(iterationStart));
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
            break;
        end
    end
    fprintf('Position update done at %1.1f\n',toc(iterationStart));
    
    % only examine the quads that have updated locations
    for k = 1:N
        if agentUpdated(k) == 1
            if norm(agentLocation(k,:)-bundle{k}(bundleWptCounter(k),:)) <= captureRadius 
                if bundleWptCounter(k) < 5
                    bundleWptCounter(k) = bundleWptCounter(k) + 1;
                end
                % packup a command packet and send it to quad
                waypointRequest.Timestamp = rostime('now');
                waypointRequest.VehicleID = k;   % Vehicle ID
                waypointRequest.CommandID = 3; % TODO: Set command ID enum in MACE
                [east, north] = F3toENU(bundle{k}(bundleWptCounter(k),1), bundle{k}(bundleWptCounter(k),2));
                waypointRequest.Easting = east; % Relative easting position to Datum
                waypointRequest.Northing = north; % Relative northing position to Datum
                waypointRequest.Altitude = agentAltitude(k);
                waypointResponse = call(waypointClient, waypointRequest, 'Timeout', 10);
            end
        end
        fprintf('Command done at %1.1f\n',toc(iterationStart));
    end
    fprintf('Iteration time before waitfor(r): %1.1f\n',toc(iterationStart));
    waitfor(r);
    iterationCounter = iterationCounter + 1;
    fprintf('Iteration %d done using %1.1f\n',iterationCounter,toc(iterationStart));
end

rosshutdown
