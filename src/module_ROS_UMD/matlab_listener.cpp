#include <stdlib.h>
#include "matlab_listener.h"

/**
 * @brief MATLABListener constructor
 */
MATLABListener::MATLABListener(const MaceCore::IModuleCommandROS* ptrRef)
{
    m_parent = ptrRef;
}


#ifdef ROS_EXISTS
bool MATLABListener::commandTakeoff(mace_matlab_msgs::CMD_TAKEOFF::Request  &req,
                                    mace_matlab_msgs::CMD_TAKEOFF::Response &res)
{
    res.success = true;

    printf("Command TAKEOFF vehicle ID: %d || sending back response: [%d]\n", req.vehicleID, res.success);

    CommandItem::SpatialTakeoff newTakeoff;
    newTakeoff.setTargetSystem(req.vehicleID);
    if(req.latitudeDeg != 0.0 && req.longitudeDeg != 0) {
        newTakeoff.position->setX(req.latitudeDeg);
        newTakeoff.position->setY(req.longitudeDeg);
    }
    newTakeoff.position->setZ(req.takeoffAlt);

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsROS* ptr){
        ptr->Event_IssueCommandTakeoff(m_parent, newTakeoff);
    });

    return true;
}

bool MATLABListener::commandArm(mace_matlab_msgs::CMD_ARM::Request  &req,
                                mace_matlab_msgs::CMD_ARM::Response &res)
{
    res.success = true;

    printf("Command ARM vehicle ID: %d || sending back response: [%d]\n", req.vehicleID, res.success);

    CommandItem::ActionArm tmpArm;
    tmpArm.setTargetSystem(req.vehicleID); // the vehicle ID corresponds to the specific vehicle //vehicle 0 is reserved for all connected vehicles
    tmpArm.setVehicleArm(req.armCmd);

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsROS* ptr){
        ptr->Event_IssueCommandSystemArm(m_parent, tmpArm);
    });


    return true;
}

bool MATLABListener::commandLand(mace_matlab_msgs::CMD_LAND::Request  &req,
                                 mace_matlab_msgs::CMD_LAND::Response &res)
{
    res.success = true;

    printf("Command LAND vehicle ID: %d || sending back response: [%d]\n", req.vehicleID, res.success);

    CommandItem::SpatialLand landCommand;
    landCommand.setTargetSystem(req.vehicleID);
    // TODO: Set generating system and coordinate frame

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsROS* ptr){
        ptr->Event_IssueCommandLand(m_parent, landCommand);
    });

    return true;
}

bool MATLABListener::commandWaypoint(mace_matlab_msgs::CMD_WPT::Request  &req,
                                     mace_matlab_msgs::CMD_WPT::Response &res)
{
    res.success = true;

    printf("Command WPT vehicle ID: %d || sending back response: [%d]\n", req.vehicleID, res.success);

    //the command here is based in a global cartesian space, we therefore need to transform it
    CartesianPosition_3D localPosition(req.easting, req.northing, req.altitude);
    GeodeticPosition_3D globalOrigin = m_parent->getDataObject()->GetGlobalOrigin();
    GeodeticPosition_3D tgtImmediate;
    DynamicsAid::LocalPositionToGlobal(globalOrigin,localPosition,tgtImmediate);

    Base3DPosition targetPosition(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT,
                                  tgtImmediate.getLongitude(), tgtImmediate.getLatitude(), tgtImmediate.getAltitude());

    CommandItem::CommandGoTo goToCommand;
    goToCommand.setTargetSystem(req.vehicleID);

    CommandItem::SpatialWaypointPtr targetWaypoint = std::make_shared<CommandItem::SpatialWaypoint>();
    targetWaypoint->setPosition(targetPosition);

    goToCommand.setSpatialCommand(targetWaypoint);
    m_parent->NotifyListeners([&](MaceCore::IModuleEventsROS* ptr) {
        ptr->Event_IssueCommandGoTo(m_parent, goToCommand);
    });

    return true;
}

bool MATLABListener::commandDatum(mace_matlab_msgs::CMD_DATUM::Request  &req,
                                  mace_matlab_msgs::CMD_DATUM::Response &res)
{
    res.success = true;

    printf("Command DATUM vehicle ID: %d || sending back response: [%d]\n", req.vehicleID, res.success);

    mace::pose::GeodeticPosition_3D origin;
    origin.setLatitude(req.latitudeDeg);
    origin.setLongitude(req.longitudeDeg);
    origin.setAltitude(0.0);

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsROS* ptr) {
        ptr->Event_SetGlobalOrigin(this, origin);
    });

    return true;
}
#endif
