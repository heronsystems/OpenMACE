rosshutdown
clear all

rosinit('NodeHost','127.0.0.1','NodeName','/TaksManager');

positionSub = rossubscriber('/MACE/UPDATE_POSITION','BufferSize', 10);% @positionCallback, 'BufferSize', 10);
% client to update waypoint
waypointClient = rossvcclient('command_waypoint');
waypointRequest = rosmessage(waypointClient);
% also need to have a bundle Sub to request bundle from main_taskgeneration

% send datum
datumClient = rossvcclient('command_datum');
datumRequest = rosmessage(datumClient);
datumRequest.CommandID = 0; % TODO: Set command ID enum in MACE
datumRequest.LatitudeDeg = 38.973699;
datumRequest.LongitudeDeg = -76.921897;
datumRequest.Timestamp = rostime('now');
datumRequest.VehicleID = 1; % Not necessary for this
datumResponse = call(datumClient, datumRequest, 'Timeout', 5);


r = robotics.Rate(4); % shall we make it to 5?
reset(r);


bundle = 2*[1 2;3 4; 5 6; 7 8; 9 10];
curWptIndex = 1;
while curWptIndex < size(bundle,1)
    tic
    msg = positionSub.LatestMessage;
    % coordinate transform UTM to F3 
    [x,y] = ENUtoF3( msg.Easting , msg.Northing );
    if norm([x,y]-bundle(curWptIndex,:)) <= 1
        curWptIndex = curWptIndex + 1;
    end
    
    waypointRequest.Timestamp = rostime('now');
    waypointRequest.VehicleID = 1;   % Vehicle ID
    waypointRequest.CommandID = 3; % TODO: Set command ID enum in MACE
    [east, north] = F3toENU(bundle(curWptIndex,1), bundle(curWptIndex,2));
    waypointRequest.Easting = east; % Relative easting position to Datum
    waypointRequest.Northing = north; % Relative northing position to Datum
    waypointRequest.Altitude = 5;
    waypointResponse = call(waypointClient, waypointRequest, 'Timeout', 10);
    
    waitfor(r);
    fprintf('Current bundle %d/%d, time %0.2f\n',curWptIndex,size(bundle,1),toc);
end

rosshutdown