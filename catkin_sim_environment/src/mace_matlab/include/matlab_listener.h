#include <stdlib.h>
#include <ros/ros.h>

#include <memory>

#include <mace_matlab/CMD_ARM.h>
#include <mace_matlab/CMD_DATUM.h>
#include <mace_matlab/CMD_LAND.h>
#include <mace_matlab/CMD_TAKEOFF.h>
#include <mace_matlab/CMD_WPT.h>

class MATLABListener
{
  public:
    MATLABListener();

    bool commandArm(mace_matlab::CMD_ARM::Request  &req, 
                    mace_matlab::CMD_ARM::Response &res);     

    bool commandDatum(mace_matlab::CMD_DATUM::Request  &req, 
                      mace_matlab::CMD_DATUM::Response &res);					

    bool commandLand(mace_matlab::CMD_LAND::Request  &req, 
                     mace_matlab::CMD_LAND::Response &res); 

    bool commandTakeoff(mace_matlab::CMD_TAKEOFF::Request  &req, 
                        mace_matlab::CMD_TAKEOFF::Response &res);					 

    bool commandWaypoint(mace_matlab::CMD_WPT::Request  &req, 
                         mace_matlab::CMD_WPT::Response &res);  
  					  
    // void takeoffCallback(const mace_matlab::Takeoff::ConstPtr &msg);


  private:

    /**
     * @brief container for a ROS node handler
     */
    ros::NodeHandle nh;

};
