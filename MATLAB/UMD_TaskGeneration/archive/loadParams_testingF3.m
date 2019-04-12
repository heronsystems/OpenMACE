function [runParams, ROS_MACE] = loadParams_testingF3()

% simulation
runParams = struct;
runParams.type = 'mace'; % 'matlab' 'mace' 'f3'
runParams.T = 60*1; % total simulation/mission time
ROS_MACE = [];


% simulation
runParams = struct;
runParams.T = 10; % total mission time
runParams.movie.useBackgroundImg = 1; %
if ( runParams.movie.useBackgroundImg )
    runParams.movie.backgroundImgFile = './data/f3_bright.png';
    runParams.movie.backgroundImgBottomLeftCornerX = -73 + 2;
    runParams.movie.backgroundImgBottomLeftCornerY = -30;
    runParams.movie.backgroundImgWidth = 108;
    runParams.movie.backgroundImgHeight = 50;
end
runParams.movie.plotF3Obstacles = 1;
if ( runParams.movie.plotF3Obstacles )
    runParams.movie.perimX = [-59 28 28 -59 -59];
    runParams.movie.perimY = [13 13 -13 -13 13];
    R = 2;
    numPts = 20;
    [runParams.movie.pole1x, runParams.movie.pole1y] = generateCircle(0, 0, R, numPts);
    [runParams.movie.pole2x, runParams.movie.pole2y] = generateCircle(-30, 0, R, numPts);
end
%ROS_MACE.ip ='10.104.206.97'; % Router at F3
%ROS_MACE.ip = '192.168.1.62'; % Kim Lab
ROS_MACE.ip = '127.0.0.1'; % localhost

ROS_MACE.coordSys = 'F3'; % 'ENU' or 'F3'
ROS_MACE.LatRef = 38.973699; % F3
ROS_MACE.LongRef = -76.921897;

% F3
ROS_MACE.LatRef = 38.973699;
ROS_MACE.LongRef = -76.921897;
ROS_MACE.startOnPerimeter = 1; % 0/1 flag
ROS_MACE.trails = 10; % length of visual trail, in no. samples

end
