function [] = global_position_test()
%GLOBAL_POSITION_TEST Summary of this function goes here
%   Detailed explanation goes here

%{
Coordinate_frame valid options are:

MAV_FRAME_GLOBAL_AMSL = 1
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
MAV_FRAME_GLOBAL_RELATIVE_TERRAIN_ALT = 5

Velocity is always in the NED direction

Valid Bitmasks (base is uint16 so all invalid : 65535)
A value of 1 indicates which values should be ignored. Must provide all 3
axis per unit
bit0:PosX, bit1:PosY, bit2:PosZ, bit3:VelX, bit4:VelY, bit5:VelZ, bit6:AccX, bit7:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate

65528 is for position, 65479 is for velocity, 65472 is position and velocity
%}
dynamicTargetClient_Kinematic = rossvcclient('command_dynamic_target_kinematic');

    dynamicTargetRequest = rosmessage(dynamicTargetClient_Kinematic);
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.VehicleID = 1; % Vehicle ID
    dynamicTargetRequest.CoordinateFrame = 3; %3 is global relative alt
    dynamicTargetRequest.XP = 149.1655330; %longitude is in the X position
    dynamicTargetRequest.YP = -35.3631790; %latitude is in the Y position
    dynamicTargetRequest.ZP = 20;
    dynamicTargetRequest.Bitmask = 65528; 
    disp('Call dynamic target command');
    waypointResponse = call(dynamicTargetClient_Kinematic, dynamicTargetRequest, 'Timeout', 5);
    pause(10);
    
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.XP = 149.1655424;
    dynamicTargetRequest.YP = -35.3629690;
    dynamicTargetRequest.ZP = 20;
    dynamicTargetRequest.Bitmask = 65528; 
    disp('Call dynamic target command');
    waypointResponse = call(dynamicTargetClient_Kinematic, dynamicTargetRequest, 'Timeout', 5);
    pause(10);
    
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.XP = 149.1652742;
    dynamicTargetRequest.YP = -35.3629679;
    dynamicTargetRequest.ZP = 20;
    dynamicTargetRequest.Bitmask = 65528; 
    disp('Call dynamic target command');
    waypointResponse = call(dynamicTargetClient_Kinematic, dynamicTargetRequest, 'Timeout', 5);
    pause(10);
    
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.XP = 149.1652688;
    dynamicTargetRequest.YP = -35.3631943;
    dynamicTargetRequest.ZP = 20;
    dynamicTargetRequest.Bitmask = 65528;
    disp('Call dynamic target command');
    waypointResponse = call(dynamicTargetClient_Kinematic, dynamicTargetRequest, 'Timeout', 5);
    pause(10);
    
end

