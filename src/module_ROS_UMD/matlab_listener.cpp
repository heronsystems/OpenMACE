#include <stdlib.h>
#include "matlab_listener.h"
#include "base/pose/pose_components.h"

/**
 * @brief MATLABListener constructor
 */
MATLABListener::MATLABListener(const MaceCore::IModuleCommandROS* ptrRef)
{
    m_parent = ptrRef;
}


#ifdef ROS_EXISTS
bool MATLABListener::commandTakeoff(mace_matlab::CMD_TAKEOFF::Request  &req,
                                    mace_matlab::CMD_TAKEOFF::Response &res)
{
    res.success = true;

    printf("Command TAKEOFF vehicle ID: %d || sending back response: [%d]\n", req.vehicleID, res.success);

    command_item::SpatialTakeoff newTakeoff;
    newTakeoff.setTargetSystem(req.vehicleID);
    mace::pose::GeodeticPosition_3D takeoffPosition;

    if(req.latitudeDeg != 0.0 && req.longitudeDeg != 0) {
        takeoffPosition.setLatitude(req.latitudeDeg);
        takeoffPosition.setLongitude(req.longitudeDeg);
    }
    takeoffPosition.setAltitudeReferenceFrame(AltitudeReferenceTypes::REF_ALT_RELATIVE);
    takeoffPosition.setAltitude(req.takeoffAlt);

    newTakeoff.setPosition(&takeoffPosition);

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsROS* ptr){
        ptr->Event_IssueCommandTakeoff(m_parent, newTakeoff);
    });

    return true;
}

bool MATLABListener::commandArm(mace_matlab::CMD_ARM::Request  &req,
                                mace_matlab::CMD_ARM::Response &res)
{
    res.success = true;

    printf("Command ARM vehicle ID: %d || sending back response: [%d]\n", req.vehicleID, res.success);

    command_item::ActionArm tmpArm;
    tmpArm.setTargetSystem(req.vehicleID); // the vehicle ID corresponds to the specific vehicle //vehicle 0 is reserved for all connected vehicles
    tmpArm.setVehicleArm(req.armCmd);

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsROS* ptr){
        ptr->Event_IssueCommandSystemArm(m_parent, tmpArm);
    });


    return true;
}

bool MATLABListener::commandLand(mace_matlab::CMD_LAND::Request  &req,
                                 mace_matlab::CMD_LAND::Response &res)
{
    res.success = true;

    printf("Command LAND vehicle ID: %d || sending back response: [%d]\n", req.vehicleID, res.success);

    command_item::SpatialLand landCommand;
    landCommand.setTargetSystem(req.vehicleID);
    // TODO: Set generating system and coordinate frame

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsROS* ptr){
        ptr->Event_IssueCommandLand(m_parent, landCommand);
    });

    return true;
}

bool MATLABListener::commandWaypoint(mace_matlab::CMD_WPT::Request  &req,
                                     mace_matlab::CMD_WPT::Response &res)
{
    res.success = true;

    printf("Command WPT vehicle ID: %d || sending back response: [%d]\n", req.vehicleID, res.success);

    //the command here is based in a global cartesian space, we therefore need to transform it
    CartesianPosition_3D localPosition(req.easting, req.northing, req.altitude);
    GeodeticPosition_3D globalOrigin = m_parent->getDataObject()->GetGlobalOrigin();
    GeodeticPosition_3D tgtImmediate;
    DynamicsAid::LocalPositionToGlobal(&globalOrigin,&localPosition,&tgtImmediate);

    mace::pose::GeodeticPosition_3D targetPosition(GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT,
                                  tgtImmediate.getLongitude(), tgtImmediate.getLatitude(),
                                  AltitudeReferenceTypes::REF_ALT_RELATIVE, tgtImmediate.getAltitude(), "");

    command_item::Action_ExecuteSpatialItem goToCommand;
    goToCommand.setTargetSystem(req.vehicleID);

    command_item::SpatialWaypoint targetWaypoint;
    targetWaypoint.setPosition(&targetPosition);

    AbstractSpatialActionPtr newPtr = std::make_shared<command_item::SpatialWaypoint>(targetWaypoint);
    goToCommand.setSpatialAction(newPtr);

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsROS* ptr) {
        ptr->Event_IssueCommandGoTo(m_parent, goToCommand);
    });

    return true;
}

bool MATLABListener::commandDatum(mace_matlab::CMD_DATUM::Request  &req,
                                  mace_matlab::CMD_DATUM::Response &res)
{
    res.success = true;

    printf("Command DATUM vehicle ID: %d || sending back response: [%d]\n", req.vehicleID, res.success);

    mace::pose::GeodeticPosition_3D origin;
    origin.setLatitude(req.latitudeDeg);
    origin.setLongitude(req.longitudeDeg);
    origin.setAltitude(0.0);

    origin.setAltitudeReferenceFrame(AltitudeReferenceTypes::REF_ALT_RELATIVE);

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsROS* ptr) {
        ptr->Event_SetGlobalOrigin(this, origin);
    });

    return true;
}

bool MATLABListener::commandDynamicTarget(mace_matlab::CMD_DYNAMIC_TARGET::Request &req,
                                          mace_matlab::CMD_DYNAMIC_TARGET::Response &res)
{
    res.success = true;

    command_item::Action_DynamicTarget dynamicTarget;
    command_target::DynamicTarget currentTarget;

    CoordinateSystemTypes currentType = mace::getCoordinateSystemType(static_cast<CoordinateFrameTypes>(req.coordinateFrame));

    if((req.bitmask&mace::pose::Position::ignoreAllPositions) == 0) //the positions are valid and should therefore use them
    {
        switch (currentType) {
        case CoordinateSystemTypes::CARTESIAN:
        {
            mace::pose::CartesianPosition_3D newPosition;
            newPosition.updatePosition(req.xP, req.yP, req.zP);
            newPosition.setCoordinateFrame(mace::getCartesianCoordinateFrame(static_cast<CoordinateFrameTypes>(req.coordinateFrame)));
            newPosition.setAltitudeReferenceFrame(mace::getAltitudeReference(newPosition.getCartesianCoordinateFrame()));
            currentTarget.setPosition(&newPosition);
            break;
        }
        case CoordinateSystemTypes::GEODETIC:
        {
            mace::pose::GeodeticPosition_3D newPosition;
            newPosition.updatePosition(req.yP, req.xP, req.zP);
            newPosition.setCoordinateFrame(mace::getGeodeticCoordinateFrame(static_cast<CoordinateFrameTypes>(req.coordinateFrame)));
            newPosition.setAltitudeReferenceFrame(mace::getAltitudeReference(newPosition.getGeodeticCoordinateFrame()));
            currentTarget.setPosition(&newPosition);
            break;
        }
        case CoordinateSystemTypes::NOT_IMPLIED:
        case CoordinateSystemTypes::UNKNOWN:
            break;
        }
    }

    if((req.bitmask&mace::pose::Velocity::ignoreAllVelocities) == 0) //the positions are valid and should therefore use them
    {
        if((currentType == CoordinateSystemTypes::CARTESIAN) || (currentType == CoordinateSystemTypes::GEODETIC))
        {
            mace::pose::Cartesian_Velocity3D newVelocity(CartesianFrameTypes::CF_LOCAL_NED);
            if(currentType == CoordinateSystemTypes::CARTESIAN)
                newVelocity.setExplicitCoordinateFrame(mace::getCartesianCoordinateFrame(static_cast<CoordinateFrameTypes>(req.coordinateFrame)));
            newVelocity.setXVelocity(req.xV);
            newVelocity.setYVelocity(req.yV);
            newVelocity.setZVelocity(req.zV);
            currentTarget.setVelocity(&newVelocity);
        }
    }

    if((req.bitmask&mace::pose::Rotation_2D::IGNORE_YAW_DIMENSION) == 0) //the positions are valid and should therefore use them
    {
        mace::pose::Rotation_2D yaw(req.yaw);
        currentTarget.setYaw(&yaw);
    }

    if((req.bitmask&mace::pose::Rotation_2D::IGNORE_YAW_RATE_DIMENSION) == 0) //the positions are valid and should therefore use them
    {
        mace::pose::Rotation_2D yawRate(req.yawRate);
        currentTarget.setYawRate(&yawRate);
    }

    dynamicTarget.setDynamicTarget(currentTarget);
    dynamicTarget.setTargetSystem(req.vehicleID);

    m_parent->NotifyListeners([&](MaceCore::IModuleEventsROS* ptr) {
        ptr->EventPP_ExecuteDynamicTarget(m_parent, dynamicTarget);
    });

    return true;
}
#endif
