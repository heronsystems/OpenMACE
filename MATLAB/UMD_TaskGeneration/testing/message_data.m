% Message Types:
% mace_matlab/CMD_ARMRequest                                     
% mace_matlab/CMD_ARMResponse                                    
% mace_matlab/CMD_DATUMRequest                                   
% mace_matlab/CMD_DATUMResponse                                  
% mace_matlab/CMD_LANDRequest                                    
% mace_matlab/CMD_LANDResponse                                   
% mace_matlab/CMD_TAKEOFFRequest                                 
% mace_matlab/CMD_TAKEOFFResponse                                
% mace_matlab/CMD_WPTRequest                                     
% mace_matlab/CMD_WPTResponse                                    
% mace_matlab/UPDATE_ATTITUDE                                    
% mace_matlab/UPDATE_BATTERY                                     
% mace_matlab/UPDATE_CMD_STATUS                                  
% mace_matlab/UPDATE_GPS                                         
% mace_matlab/UPDATE_HEARTBEAT                                   
% mace_matlab/UPDATE_POSITION                                    
% mace_matlab/UPDATE_VEHICLE_TARGET 

positionSub = rossubscriber('/MACE/UPDATE_POSITION', @positionCallback, 'BufferSize', 10);
%     MessageType: 'mace_matlab/UPDATE_POSITION'
%       Timestamp: [1×1 Time]
%       VehicleID: 0
%        Northing: 0
%         Easting: 0
%        Altitude: 0
%      NorthSpeed: 0
%       EastSpeed: 0

attitudeSub = rossubscriber('/MACE/UPDATE_ATTITUDE', @attitudeCallback, 'BufferSize', 10)
%     MessageType: 'mace_matlab/UPDATE_ATTITUDE'
%       Timestamp: [1×1 Time]
%       VehicleID: 0
%            Roll: 0
%           Pitch: 0
%             Yaw: 0
%         YawRate: 0
%       PitchRate: 0
%        RollRate: 0

gpsSub = rossubscriber('/MACE/UPDATE_GPS', @gpsCallback, 'BufferSize', 10);
%     MessageType: 'mace_matlab/UPDATE_GPS'
%       Timestamp: [1×1 Time]
%       VehicleID: 0
%          GpsFix: ''
%         NumSats: 0
%            Hdop: 0
%            Vdop: 0

heartbeatSub = rossubscriber('/MACE/UPDATE_HEARTBEAT', @heartbeatCallback, 'BufferSize', 10);
%      MessageType: 'mace_matlab/UPDATE_HEARTBEAT'
%        Timestamp: [1×1 Time]
%        VehicleID: 0
%        Autopilot: ''
%     AircraftType: ''
%      IsCompanion: 0
%     MissionState: ''
%   Use showdetails to show the contents of the message

vehicleTargetSub = rossubscriber('/MACE/TARGET_STATUS',@vehicleTargetCallback, 'BufferSize', 10);
%     MessageType: 'mace_matlab/UPDATE_VEHICLE_TARGET'
%       Timestamp: [1×1 Time]
%       VehicleID: 0
%        Northing: 0
%         Easting: 0
%        Altitude: 0
%        Distance: 0
%           State: 0