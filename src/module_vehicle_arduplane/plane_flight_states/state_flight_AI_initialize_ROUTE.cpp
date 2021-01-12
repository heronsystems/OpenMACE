#include "state_flight_AI_initialize_ROUTE.h"
namespace ardupilot {
namespace state{
AP_State_FlightAI_Initialize_ROUTE::AP_State_FlightAI_Initialize_ROUTE():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ROUTE)
{
}
void AP_State_FlightAI_Initialize_ROUTE::OnExit()
{
    dynamic_cast<VehicleObject_Arduplane*>(&Owner())->shutdownTargetTracking();

    Owner().state->m_CompleteVehicleCarState.RemoveNotifier(this);

    if(Owner().ControllersCollection()->Exist("TestInitialization_PositionController"))
        Owner().ControllersCollection()->Remove("TestInitialization_PositionController");
    if(Owner().ControllersCollection()->Exist("TestInitialization_SpeedController"))
    {
        MAVLINKUXVControllers::Command_ChangeSpeed* speedController = static_cast<MAVLINKUXVControllers::Command_ChangeSpeed*>(Owner().ControllersCollection()->At("TestInitialization_SpeedController"));
        speedController->Shutdown();
    }
    if(m_GeodeticCommsController)
        delete m_GeodeticCommsController;
    AbstractStateArdupilot::OnExit();

}
AbstractStateArdupilot* AP_State_FlightAI_Initialize_ROUTE::getClone() const
{
    return (new AP_State_FlightAI_Initialize_ROUTE(*this));
}
void AP_State_FlightAI_Initialize_ROUTE::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightAI_Initialize_ROUTE(*this);
}
hsm::Transition AP_State_FlightAI_Initialize_ROUTE::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();
    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        case Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ABORT:
        {
            rtn = hsm::SiblingTransition<AP_State_FlightAI_Initialize_ABORT>();
            break;
        }
        default:
            std::cout << "I dont know how we ended up in this transition state from STATE_FLIGHT_GUIDED. STATE: " << MACEHSMStateToString(_desiredState) << std::endl;
            break;
        }
    }
    return rtn;
}
bool AP_State_FlightAI_Initialize_ROUTE::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {
    default:
        break;
    } //end of switch statement command type
    return success;
}
void AP_State_FlightAI_Initialize_ROUTE::Update()
{
}
void AP_State_FlightAI_Initialize_ROUTE::OnEnter()
{
}
void AP_State_FlightAI_Initialize_ROUTE::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    //If we have a command we probably will be entering a state let us figure it out
    this->OnEnter();
}
void AP_State_FlightAI_Initialize_ROUTE::OnEnter(const command_item::Action_InitializeTestSetup &initialization)
{
    m_SetupConditions = initialization;
    setupTrackingControllers();
    setupAircraftSpeed();
    setupAircraftRoute();
}
void AP_State_FlightAI_Initialize_ROUTE::setupTrackingControllers()
{
    //Insert a new controller only one time in the guided state to manage the entirity of the commands that are of the dynamic target type
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
    m_GeodeticCommsController = new MAVLINKUXVControllers::ControllerGuidedTargetItem_WP(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    collection->Insert("TestInitialization_PositionController",m_GeodeticCommsController);
    dynamic_cast<VehicleObject_Arduplane*>(&Owner())->m_TargetController->connectVehicleTargetCallback(AP_State_FlightAI_Initialize_ROUTE::staticCallbackFunction_VehicleTarget, this);
}
void AP_State_FlightAI_Initialize_ROUTE::setupAircraftSpeed()
{
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
    //Insert a new controller only one time in the guided state to manage the entirity of the commands that are of the dynamic target type
    MAVLINKUXVControllers::Command_ChangeSpeed* speedCommsController = new MAVLINKUXVControllers::Command_ChangeSpeed(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    speedCommsController->AddLambda_Finished(this, [this, speedCommsController](const bool completed, const uint8_t finishCode){
        UNUSED(completed);
        UNUSED(finishCode);
        speedCommsController->Shutdown();
    });
    speedCommsController->setLambda_Shutdown([this]()
    {
        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
        auto ptr = collection->Remove("TestInitialization_SpeedController");
        delete ptr;
    });
    collection->Insert("TestInitialization_SpeedController",speedCommsController);
    MavlinkEntityKey target = Owner().getMAVLINKID();
    MavlinkEntityKey sender = 255;
    command_item::ActionChangeSpeed commandSpeed(static_cast<unsigned int>(sender),static_cast<unsigned int>(target));
    commandSpeed.setDesiredSpeed(m_SetupConditions.m_InitialConditions.getSpeed());
    speedCommsController->Send(commandSpeed, sender, target);
}
void AP_State_FlightAI_Initialize_ROUTE::setupAircraftRoute()
{
    MissionItem::MissionList flightProfile = routeViaDubinsSpline();
//    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().GlobalControllersCollection();
//    if(collection->Exist("missionController"))
//    {
//        mace::pose::GeodeticPosition_3D currentHome = Owner().environment->vehicleGlobalHome.get();
//        command_item::SpatialHome routeHome(Owner().getMAVLINKID(),Owner().getMAVLINKID());
//        routeHome.setPosition(&currentHome);
//        MAVLINKUXVControllers::ControllerMission* missionController = static_cast<MAVLINKUXVControllers::ControllerMission*>(collection->At("missionController"));
//        missionController->AddLambda_Finished(this, [this, routeHome, flightProfile](const bool completed, const uint8_t &code){
//            if(completed && (code == MAV_MISSION_RESULT::MAV_MISSION_ACCEPTED))
//            {
//                //////////////////////////////
//                ///Update about Home position
//                //////////////////////////////
//                Owner().publishVehicleHome(Owner().getMAVLINKID(), routeHome);
//                //////////////////////////////
//                ///Update about mission list
//                //////////////////////////////
//                Owner().publishVehicleAutoMission(Owner().getMAVLINKID(), flightProfile);
//                //Now we can start tracking
//            }
//            else
//            {
//                //the upload failed, we are going to abort this run
//                setDesiredStateEnum(Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ABORT);
//            }
//        });
//        missionController->UploadMission(flightProfile,routeHome,Owner().getMAVLINKID());
//    }
}
MissionItem::MissionList AP_State_FlightAI_Initialize_ROUTE::routeViaDubinsSpline()
{
    MissionItem::MissionList setupMission(Owner().getMAVLINKID(),Owner().getMAVLINKID(),MISSIONTYPE::AUTO, MISSIONSTATE::CURRENT);
    command_item::SpatialWaypointPtr wpTarget = std::make_shared<command_item::SpatialWaypoint>(Owner().getMAVLINKID(),Owner().getMAVLINKID());
    //Setup the vehicle input into the dubins spline
    //We are going to receive the local state as indicated from the vehicle in the NED frame
    VehicleState_Cartesian3D currentState = Owner().state->m_CompleteVehicleCarState.get();
    Eigen::Vector3d dataENU = ftf::transform_frame_ned_enu(currentState.m_Position.getDataVector());
    currentState.m_Position.setCoordinateFrame(CartesianFrameTypes::CF_LOCAL_ENU);
    currentState.m_Position.updatePosition(dataENU(0),dataENU(1),dataENU(2));
    Eigen::Quaterniond NEDorientation = currentState.m_Rotation.getQuaternion();
    Eigen::Quaterniond ENUorientation = ftf::transform_orientation_aircraft_baselink(
                ftf::transform_orientation_ned_enu(NEDorientation));
    double yawOrientation_ENU = ftf::quaternion_get_yaw(ENUorientation);
    double q0[] = { currentState.m_Position.getXPosition(), currentState.m_Position.getYPosition(), yawOrientation_ENU};
    //Setup the target position from which we are going to start the test
    //First let us setup the target position
    pose::GeodeticPosition_3D targetGeoPosition = m_SetupConditions.m_InitialConditions.getPosition();
    double targetAlt = targetGeoPosition.getAltitude();
    //Grab the local reference origin for the aircraft
    pose::GeodeticPosition_3D referenceOrigin = Owner().environment->vehicleGlobalOrigin.get();
    referenceOrigin.setCoordinateFrame(GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT);
    referenceOrigin.setAltitudeReferenceFrame(AltitudeReferenceTypes::REF_ALT_RELATIVE);
    referenceOrigin.setAltitude(0.0);

    //Generate the target cartesian position from the geodetic target coordinates
    pose::CartesianPosition_3D targetENUCarPosition, projectedTarget;
    DynamicsAid::GlobalPositionToLocal(&referenceOrigin, &targetGeoPosition, &targetENUCarPosition);
    targetENUCarPosition.setCoordinateFrame(CartesianFrameTypes::CF_LOCAL_ENU);
    targetENUCarPosition.setAltitude(targetAlt);
    projectedTarget = targetENUCarPosition;

    //Second, let us setup the target orientation
    Eigen::Quaterniond targetNEDOrientation = m_SetupConditions.m_InitialConditions.getRotation().getQuaternion();
    Eigen::Quaterniond targetENUOrientation = ftf::transform_orientation_aircraft_baselink(
                ftf::transform_orientation_ned_enu(targetNEDOrientation));
    double targetOrientation_yaw = ftf::quaternion_get_yaw(targetENUOrientation);
    double revTargetOrientation_yaw = math::reverseBearing(targetOrientation_yaw);

    //Third, let us project the target state to be 5x the target speed of the aircraft behind. This should alot 2 seconds
    //of straight and level fight to runup into the target state
    projectedTarget.applyPositionalShiftFromPolar(5*m_SetupConditions.m_InitialConditions.getSpeed(),revTargetOrientation_yaw); //behind the target
    double q1[] = { projectedTarget.getXPosition(), projectedTarget.getYPosition(), targetOrientation_yaw};

    DubinsPath path;
    VectorStateQueue targetQueue;
    VehiclePath_Linear linearPath;

    std::vector<pose::GeodeticPosition_3D> pathGeo;

    dubins_shortest_path( &path, q0, q1, Owner().m_AgentParams.getTurningRadius());
    dubins_path_sample_many(&path, Owner().m_AgentParams.getSamplingDistance(), targetQueue, targetAlt);

    for(size_t i = 0; i < targetQueue.size(); i++)
    {
        pose::GeodeticPosition_3D transformedPoint;
        pose::CartesianPosition_3D point; point.updateFromDataVector(targetQueue.at(i)._position);
        DynamicsAid::LocalPositionToGlobal(&referenceOrigin, &point, &transformedPoint);
        pathGeo.push_back(transformedPoint);
        wpTarget->setPosition(&transformedPoint);
        setupMission.insertMissionItem(wpTarget);
    }

    //Lastly, add the actual target to the appended list
    TrajectoryPoint trajPoint;
    targetENUCarPosition.updatePosition(0.0,0.0,0.0);
    pathGeo.push_back(targetGeoPosition);
    wpTarget->setPosition(&targetGeoPosition);
    setupMission.insertMissionItem(wpTarget);
    DynamicsAid::GlobalPositionToLocal(&referenceOrigin, &targetGeoPosition, &targetENUCarPosition);
    trajPoint._position = targetENUCarPosition.getDataVector();
    targetQueue.push_back(trajPoint);

    targetGeoPosition.applyPositionalShiftFromPolar(5*m_SetupConditions.m_InitialConditions.getSpeed(),targetOrientation_yaw); //ahead of the target
    pathGeo.push_back(targetGeoPosition);
    wpTarget->setPosition(&targetGeoPosition);
    setupMission.insertMissionItem(wpTarget);
    targetENUCarPosition.updatePosition(0.0,0.0,0.0);
    DynamicsAid::GlobalPositionToLocal(&referenceOrigin, &targetGeoPosition, &targetENUCarPosition);
    trajPoint._position = targetENUCarPosition.getDataVector();
    targetQueue.push_back(trajPoint);

    linearPath.setVertices(pathGeo);

    Owner().publishVehicleTrajectory(Owner().getMAVLINKID(), linearPath);

    dynamic_cast<VehicleObject_Arduplane*>(&Owner())->m_TrackingManager->receivedNewTrajectory(targetQueue);

    return setupMission;
}
MissionItem::MissionList AP_State_FlightAI_Initialize_ROUTE::routeStraightVectors()
{
    MissionItem::MissionList setupMission(Owner().getMAVLINKID(),Owner().getMAVLINKID(),MISSIONTYPE::AUTO, MISSIONSTATE::CURRENT);
    command_item::SpatialWaypointPtr wpTarget = std::make_shared<command_item::SpatialWaypoint>(Owner().getMAVLINKID(),Owner().getMAVLINKID());
    pose::GeodeticPosition_3D targetGeoPosition = m_SetupConditions.m_InitialConditions.getPosition();
    double targetAlt = targetGeoPosition.getAltitude();
    pose::GeodeticPosition_3D referenceOrigin = Owner().environment->vehicleGlobalOrigin.get();
    referenceOrigin.setCoordinateFrame(GeodeticFrameTypes::CF_GLOBAL_RELATIVE_ALT);
    referenceOrigin.setAltitudeReferenceFrame(AltitudeReferenceTypes::REF_ALT_RELATIVE);
    referenceOrigin.setAltitude(0.0);
    pose::CartesianPosition_3D targetENUCarPosition;
    DynamicsAid::GlobalPositionToLocal(&referenceOrigin, &targetGeoPosition, &targetENUCarPosition);
    targetENUCarPosition.setCoordinateFrame(CartesianFrameTypes::CF_LOCAL_ENU);
    targetENUCarPosition.setAltitude(targetAlt);
    pose::CartesianPosition_3D projectedTarget = targetENUCarPosition;
    Eigen::Quaterniond targetNEDOrientation = m_SetupConditions.m_InitialConditions.getRotation().getQuaternion();
    Eigen::Quaterniond targetENUOrientation = ftf::transform_orientation_aircraft_baselink(
                ftf::transform_orientation_ned_enu(targetNEDOrientation));

    double targetYaw = ftf::quaternion_get_yaw(targetENUOrientation);
    double revTargetYaw = math::reverseBearing(targetYaw);

    VectorStateQueue targetQueue; TrajectoryPoint targetPoint;

    projectedTarget.applyPositionalShiftFromPolar(40,targetYaw); //ahead of the target
    targetPoint._position = projectedTarget.getDataVector();
    targetQueue.push_front(targetPoint);

    projectedTarget.applyPositionalShiftFromPolar(40,revTargetYaw); //at the target
    targetPoint._position = projectedTarget.getDataVector();
    targetQueue.push_front(targetPoint);

    projectedTarget.applyPositionalShiftFromPolar(40,revTargetYaw); //behind the target
    targetPoint._position = projectedTarget.getDataVector();
    targetQueue.push_front(targetPoint);

    projectedTarget.applyPositionalShiftFromPolar(40,revTargetYaw); //at the target
    targetPoint._position = projectedTarget.getDataVector();
    targetQueue.push_front(targetPoint);

    for(size_t i = 0; i < targetQueue.size(); i++)
    {
        pose::GeodeticPosition_3D transformedPoint;
        pose::CartesianPosition_3D point; point.updateFromDataVector(targetQueue.at(i)._position);
        DynamicsAid::LocalPositionToGlobal(&referenceOrigin, &point, &transformedPoint);
        wpTarget->setPosition(&transformedPoint);
        setupMission.insertMissionItem(wpTarget);
    }
    return setupMission;
}
void AP_State_FlightAI_Initialize_ROUTE::callbackFunction_VehicleTarget(const pose::GeodeticPosition_3D &projection, const pose::GeodeticPosition_3D &actual)
{
    MavlinkEntityKey target = Owner().getMAVLINKID();
    MavlinkEntityKey sender = 255;
    command_item::SpatialWaypoint targetWaypoint(static_cast<unsigned int>(sender), static_cast<unsigned int>(target));
    targetWaypoint.setPosition(&projection);
    m_GeodeticCommsController->Broadcast(targetWaypoint, sender);
}
void AP_State_FlightAI_Initialize_ROUTE::executeFrameComparison()
{
    std::cout<<"---Comparison of local position data---"<<std::endl;
    pose::CartesianPosition_3D currentLocal = Owner().state->vehicleLocalPosition.get();
    pose::GeodeticPosition_3D currentGlobal = Owner().state->vehicleGlobalPosition.get();
    pose::CartesianPosition_3D transformedOrigin, transformedHome;
    pose::GeodeticPosition_3D globalHome, globalOrigin;
    if(Owner().environment->vehicleGlobalHome.hasBeenSet())
    {
        globalHome = Owner().environment->vehicleGlobalHome.get();
        globalHome.setAltitudeReferenceFrame(AltitudeReferenceTypes::REF_ALT_RELATIVE);
        //LET US DO A COMPARISON WITH THE HOME POSITION
        DynamicsAid::GlobalPositionToLocal(&globalHome, &currentGlobal, &transformedHome);
    }
    if(Owner().environment->vehicleGlobalOrigin.hasBeenSet())
    {
        globalOrigin = Owner().environment->vehicleGlobalOrigin.get();
        globalOrigin.setAltitudeReferenceFrame(AltitudeReferenceTypes::REF_ALT_RELATIVE);
        //LET US DO A COMPARISON WITH THE HOME POSITION
        DynamicsAid::GlobalPositionToLocal(&globalOrigin, &currentGlobal, &transformedOrigin);
    }
    std::cout<<currentLocal<<std::endl;
    std::cout<<transformedHome<<std::endl;
    std::cout<<transformedOrigin<<std::endl;
    std::cout<<"------------------------"<<std::endl;
}

void AP_State_FlightAI_Initialize_ROUTE::handleTestProcedural(const command_item::Action_ProceduralCommand &command)
{
    std::cout<<"We have received a procedural command while in the route state!"<<std::endl;
    switch (command.whatIsTheProcedural()) {
    case AI_PROCEDURAL_COMMANDS::STOP:
    {
        setDesiredStateEnum(Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ABORT);
        break;
    }
    case AI_PROCEDURAL_COMMANDS::ABORT:
    {
        setDesiredStateEnum(Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ABORT);
        break;
    }
    default:
    {
        break;
    }

    } //end of switch statement
}

} //end of namespace ardupilot
} //end of namespace state
#include "plane_flight_states/state_flight_AI_initialize_ABORT.h"
