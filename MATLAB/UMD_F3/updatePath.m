function updatePath(varargin)
% Note: make sure to run this script from the root folder so the following
% paths can be added correctly

 celldisp(varargin)

if ( length(varargin)==2 )
   cd(varargin{1}); 
end

%addpath('/home/wolek/Desktop/Research/Projects/UMD/Heron/OpenMACE/catkin_sim_environment/src/matlab_gen/msggen')

addpath('./subroutines')
% addpath('./subroutines/tracking')
% addpath('./subroutines/tasking')
% addpath('./subroutines/sensors')
addpath('./subroutines/ROS')
% addpath('./subroutines/mapping')
% addpath('./subroutines/misc')
addpath('./subroutines/modules')
addpath('./subroutines/geometry')
addpath('./subroutines/F3')
addpath('./subroutines/environment')
addpath('./subroutines/XBee/');
% addpath('./subroutines/shortcuts')
% addpath('./subroutines/dynamics')
% addpath('./subroutines/display')
% addpath('./subroutines/display/init')
% addpath('./subroutines/display/update')
%
% addpath('./external/munkres')
addpath('./external/utm2deg')
addpath('./external/deg2utm')
% addpath('./external/lat_lon_proportions')
% addpath('./external/plotmd')
% addpath('./external/openstreetmap')
% addpath('./external/geodetic299/geodetic')
% addpath('./external/xml2struct')
% addpath('./external/brewer')
%
addpath('./data')

if ( length(varargin)==2)
   cd(varargin{2}); 
end
end
