Configuring simulated quadrotors with ArduCopter parameters

To configure the ArduCopter parameters of the simulated quadrotors, we only need to append the 'parameter name'-'parameter value' pair to the end of the configuration file.

For example, we want to configure quad 0 to have 50 cm/s horizontal waypoint speed and 100 cm capture radius for reaching waypoints. Open copter_0.parm under the path
/ardupilot/Tools/autotest/default_params/MACE_params

and type the following to lines to the end of the file:

# Quadrotor speed 
WPNAV_SPEED 50
# Quadrotor capture radius 
WPNAV_RADIUS 100

Note that in order to configure the quadrotor numbered x, the corresponding parameters have to be set in the file copter_x.parm under the path shown above.