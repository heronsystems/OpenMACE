function resp = ROSBundleServer(~,req,resp)

global bundleSource

numVehicle = req.NumVehicle; % total number of vehicles

if isempty(bundleSource)
    source = zeros(12);
else
    source = bundleSource;
end
% source
resp.BundleID = source(1,1);
for k = 1:numVehicle
    eval(['resp.Agent' num2str(k) 'E1 = source(k,3);']);
    eval(['resp.Agent' num2str(k) 'N1 = source(k,4);']);
    eval(['resp.Agent' num2str(k) 'E2 = source(k,5);']);
    eval(['resp.Agent' num2str(k) 'N2 = source(k,6);']);
    eval(['resp.Agent' num2str(k) 'E3 = source(k,7);']);
    eval(['resp.Agent' num2str(k) 'N3 = source(k,8);']);
    eval(['resp.Agent' num2str(k) 'E4 = source(k,9);']);
    eval(['resp.Agent' num2str(k) 'N4 = source(k,10);']);
    eval(['resp.Agent' num2str(k) 'E5 = source(k,11);']);
    eval(['resp.Agent' num2str(k) 'N5 = source(k,12);']);
    eval(['resp.Agent' num2str(k) 'Al = source(k,2);']);
end
resp.BundleStatus = source(end,1); % load the status of the bundle (0 for 'written in process' and 1 for 'written complete')
% resp.Altitude1 = source(VehicleID,2);
% resp.Altitude2 = source(VehicleID,2);
% resp.Altitude3 = source(VehicleID,2);
% resp.Altitude4 = source(VehicleID,2);
% resp.Altitude5 = source(VehicleID,2);
% 
% resp.Easting1 = source(VehicleID,3);
% resp.Northing1 = source(VehicleID,4);
% resp.Easting2 = source(VehicleID,5);
% resp.Northing2 = source(VehicleID,6);
% resp.Easting3 = source(VehicleID,7);
% resp.Northing3 = source(VehicleID,8);
% resp.Easting4 = source(VehicleID,9);
% resp.Northing4 = source(VehicleID,10);
% resp.Easting5 = source(VehicleID,11);
% resp.Northing5 = source(VehicleID,12);

