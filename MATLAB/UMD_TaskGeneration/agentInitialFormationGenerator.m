clear; close all; clc;
format compact;
updatePath;


for k = 1:10

[runParams, ROS_MACE, trueWorld, swarmModel, targetModel] = loadParams_osm();

[xv, yv, ~] = distributeUniformlyAlongCurve(swarmModel.N,trueWorld.xpoly,trueWorld.ypoly);

clearvars -except xv yv k

save(['./scenes/initialFormation' num2str(k) '.mat'],'xv','yv');

end