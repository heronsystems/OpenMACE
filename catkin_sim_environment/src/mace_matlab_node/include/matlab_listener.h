#include <stdlib.h>
#include <ros/ros.h>

#include <memory>

#include <mace_matlab_msgs/CMD_ARM.h>
#include <mace_matlab_msgs/CMD_DATUM.h>
#include <mace_matlab_msgs/CMD_LAND.h>
#include <mace_matlab_msgs/CMD_TAKEOFF.h>
#include <mace_matlab_msgs/CMD_WPT.h>

class MATLABListener
{
  public:
    MATLABListener();

    bool commandArm(mace_matlab_msgs::CMD_ARM::Request  &req,
                    mace_matlab_msgs::CMD_ARM::Response &res);

    bool commandDatum(mace_matlab_msgs::CMD_DATUM::Request  &req,
                      mace_matlab_msgs::CMD_DATUM::Response &res);

    bool commandLand(mace_matlab_msgs::CMD_LAND::Request  &req,
                     mace_matlab_msgs::CMD_LAND::Response &res);

    bool commandTakeoff(mace_matlab_msgs::CMD_TAKEOFF::Request  &req,
                        mace_matlab_msgs::CMD_TAKEOFF::Response &res);

    bool commandWaypoint(mace_matlab_msgs::CMD_WPT::Request  &req,
                         mace_matlab_msgs::CMD_WPT::Response &res);

    // void takeoffCallback(const mace_matlab::Takeoff::ConstPtr &msg);


  private:

    /**
     * @brief container for a ROS node handler
     */
    ros::NodeHandle nh;

};
