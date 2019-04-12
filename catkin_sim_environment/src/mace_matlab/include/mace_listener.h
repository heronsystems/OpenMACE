#include <stdlib.h>
#include <ros/ros.h>

#include <memory>

#include <mace_matlab/UPDATE_ATTITUDE.h>
#include <mace_matlab/UPDATE_BATTERY.h>
#include <mace_matlab/UPDATE_CMD_STATUS.h>
#include <mace_matlab/UPDATE_GPS.h>
#include <mace_matlab/UPDATE_HEARTBEAT.h>
#include <mace_matlab/UPDATE_POSITION.h>
#include <mace_matlab/UPDATE_VEHICLE_TARGET.h>

class MACEListener
{
public:
    MACEListener();
    
    void attitudeCallback(const mace_matlab::UPDATE_ATTITUDE::ConstPtr &msg);

    void batteryCallback(const mace_matlab::UPDATE_BATTERY::ConstPtr &msg);

    // Not needed: services require a response at time of send
    void cmdStatusCallback(const mace_matlab::UPDATE_CMD_STATUS::ConstPtr &msg);

    void gpsCallback(const mace_matlab::UPDATE_GPS::ConstPtr &msg);

    void heartbeatCallback(const mace_matlab::UPDATE_HEARTBEAT::ConstPtr &msg);

    void positionCallback(const mace_matlab::UPDATE_POSITION::ConstPtr &msg);

    void vehicleTargetCallback(const mace_matlab::UPDATE_VEHICLE_TARGET::ConstPtr &msg);

private:

    /**
     * @brief container for a ROS node handler
     */
    ros::NodeHandle nh;

};
