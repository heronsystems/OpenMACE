function kinematicLocalCommand(ROS_MACE,agentID,px,py,pz,posCF,vx,vy,vz,velCF,yaw,yawRate)

% ROS_MACE
% agentID
% px,py,pz
% posCF: 'ENU' (inertial), 'ENUOFFSET' (offset in ENU), or 'RFU' (right-forward-up in body frame)
% vx,vy,vz
% velCF: 'ENU' (inertial) or 'RFU' (right-forward-up in body frame)
% yaw, yawRate
dynamicTargetRequest = rosmessage(ROS_MACE.kinematicTargetClient);
dynamicTargetRequest.Timestamp = rostime('now');
dynamicTargetRequest.VehicleID = agentID; % Vehicle ID

% use the combination of coordinate frame to determine the number here
switch [posCF velCF]
    case 'ENUENU'
        dynamicTargetRequest.CoordinateFrame = 11;
    case 'ENUOFFSETENU'
        dynamicTargetRequest.CoordinateFrame = 13;
    case 'ENURFU'
        dynamicTargetRequest.CoordinateFrame = 15;
    case 'RFURFU'
        dynamicTargetRequest.CoordinateFrame = 17;
    otherwise
        fprintf('Wrong coordinate frame!\n');
        fprintf('PositionCF = %c, velocityCF = %c.\n',posCF, velCF);
        fprintf('Discard command!\n');
        return;
end

% generate bitmask and transform the coordinates
bitmask = [];
if ~isempty(px) % px, py, and pz are taken as a pair
    bitmask = ['000' bitmask];
    if strcmp(posCF,'ENU') || strcmp(posCF,'ENUOFFSET')
        [P_NEU_E,P_NEU_N] = F3toENU(px,py);
        dynamicTargetRequest.XP = P_NEU_N;
        dynamicTargetRequest.YP = P_NEU_E;
    else
        dynamicTargetRequest.XP = py;
        dynamicTargetRequest.YP = px;
    end
    dynamicTargetRequest.ZP = -pz;
else
    bitmask = ['111' bitmask];
end


if ~isempty(vx) % vx, vy, and vz are taken as a pair
    bitmask = ['000' bitmask];
    if strcmp(velCF,'ENU')
        [V_NEU_E,V_NEU_N] = F3toENU(vx,vy);
        dynamicTargetRequest.XV = V_NEU_N;
        dynamicTargetRequest.YV = V_NEU_E;
    else
        dynamicTargetRequest.XV = vy;
        dynamicTargetRequest.YV = vx;
    end
    dynamicTargetRequest.ZV = -vz;
else
    bitmask = ['111' bitmask];
end

bitmask = ['1111' bitmask]; % acceleration commands not used here

if ~isempty(yaw)
    bitmask = ['0' bitmask];
    dynamicTargetRequest.Yaw = -yaw+2.3736; % in radian
else
    bitmask = ['1' bitmask];
end
if ~isempty(yawRate)
    bitmask = ['0' bitmask];
    dynamicTargetRequest.YawRate = -yawRate;
else
    bitmask = ['1' bitmask];
end

bitmask = ['1111' bitmask];

dynamicTargetRequest.Bitmask = bin2dec(bitmask);
%                                           ['1111101111111111']
%                                          '1111101111000111'
%                                         ['1111011111111111']
% dynamicTargetRequest.Bitmask = bin2dec('1111101111000111');
% %                                          || | ||||||||
% %                                          || | |||||||`bit0: px
% %                                          || | ||||||`bit1: py
% %                                          || | |||||`bit2: pz
% %                                          || | ||||`bit3: vx
% %                                          || | |||`bit4: vy
% %                                          || | ||`bit5: vz
% %                                          || | |`bit6: ax (not used)
% %                                          || | `bit7: ay (not used)
% %                                          || `bit9: az (not used)
% %                                          |`bit11: yaw
% %                                          `bit12: yaw rate

% printout message about the message sent.
% fprintf('Kinematic command sent.\n');
waypointResponse = call(ROS_MACE.kinematicTargetClient, dynamicTargetRequest, 'Timeout', 5);


end