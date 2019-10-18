function [] = local_position_test()
%LOCAL_POSITION_TEST Summary of this function goes here
%   Detailed explanation goes here

%{
MAV_FRAME_LOCAL_NED = 11
Positions are relative to the vehicle�s EKF Origin in NED frame

I.e x=1,y=2,z=3 is 1m North, 2m East and 3m Down from the origin

The EKF origin is the vehicle�s location when it first achieved a good position estimate

Velocity are in NED frame

MAV_FRAME_LOCAL_OFFSET_NED = 13
Positions are relative to the vehicle�s current position

I.e. x=1,y=2,z=3 is 1m North, 2m East and 3m below the current position.

Velocity are in NED frame.

MAV_FRAME_BODY_OFFSET_NED = 17
Positions are relative to the vehicle�s current position and heading

I.e x=1,y=2,z=3 is 1m forward, 2m right and 3m Down from the current position

Velocities are relative to the current vehicle heading. Use this to specify the speed forward, right and down (or the opposite if you use negative values).

MAV_FRAME_BODY_NED = 15
Positions are relative to the EKF Origin in NED frame

I.e x=1,y=2,z=3 is 1m North, 2m East and 3m Down from the origin

Velocities are relative to the current vehicle heading. Use this to specify the speed forward, right and down (or the opposite if you use negative values).

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
    dynamicTargetRequest.CoordinateFrame = 15; 
    dynamicTargetRequest.XP = 0;
    dynamicTargetRequest.YP = 0;
    dynamicTargetRequest.ZP = -10;
    dynamicTargetRequest.XV = 0;
    dynamicTargetRequest.YV = 0;
    dynamicTargetRequest.ZV = 0;
    dynamicTargetRequest.Bitmask = 65528; %65528 is for position, 65479 is for velocity, 65472 is position and velocity
    
    %waypointResponse = call(waypointClient, waypointRequest, 'Timeout', 5);
    disp('Call dynamic target command');
    waypointResponse = call(dynamicTargetClient_Kinematic, dynamicTargetRequest, 'Timeout', 5);
    pause(10);
    
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.XP = 0;
    dynamicTargetRequest.YP = 10;
    dynamicTargetRequest.ZP = 0;
    dynamicTargetRequest.Bitmask = 65528; %65528 is for position, 65479 is for velocity, 65472 is position and velocity
    disp('Call dynamic target command');
    waypointResponse = call(dynamicTargetClient_Kinematic, dynamicTargetRequest, 'Timeout', 5);
    pause(10);
    
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.XP = 10;
    dynamicTargetRequest.YP = 0;
    dynamicTargetRequest.ZP = 0;
    dynamicTargetRequest.Bitmask = 65528; %65528 is for position, 65479 is for velocity, 65472 is position and velocity
    disp('Call dynamic target command');
    waypointResponse = call(dynamicTargetClient_Kinematic, dynamicTargetRequest, 'Timeout', 5);
    pause(10);
    
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.XP = 0;
    dynamicTargetRequest.YP = -10;
    dynamicTargetRequest.ZP = 0;
    dynamicTargetRequest.Bitmask = 65528; %65528 is for position, 65479 is for velocity, 65472 is position and velocity
    disp('Call dynamic target command');
    waypointResponse = call(dynamicTargetClient_Kinematic, dynamicTargetRequest, 'Timeout', 5);
    pause(10);
    
    dynamicTargetRequest.Timestamp = rostime('now');
    dynamicTargetRequest.XP = -10;
    dynamicTargetRequest.YP = 0;
    dynamicTargetRequest.ZP = 0;
    dynamicTargetRequest.Bitmask = 65528; %65528 is for position, 65479 is for velocity, 65472 is position and velocity
    disp('Call dynamic target command');
    waypointResponse = call(dynamicTargetClient_Kinematic, dynamicTargetRequest, 'Timeout', 5);
    pause(10);
    
end

