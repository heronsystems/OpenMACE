#include "module_path_planning_nasaphase2.h"

#include "mace_core/module_factory.h"

#include <iostream>
#include <typeinfo>

ModulePathPlanningNASAPhase2::ModulePathPlanningNASAPhase2() :
    MaceCore::IModuleCommandPathPlanning(),
    m_Space(nullptr),
    sampler(nullptr),
    stateCheck(nullptr),
    spaceInfo(nullptr),
    m_VehicleDataTopic("vehicleData"),
    m_MissionDataTopic("vehicleMission"),
    m_PlanningStateTopic("planningState"),
    m_MapTopic("mappingData")
{    

    myfile.open("C:/Github/OpenMACE/potential_fields_velocity_response.csv");

    m_Space = std::make_shared<state_space::Cartesian2DSpace>();

    sampler = std::make_shared<mace::state_space::Cartesian2DSpace_Sampler>(m_Space);

    stateCheck = std::make_shared<mace::state_space::SpecialValidityCheck>(m_Space);
    auto stateValidityCheck = ([this](const mace::state_space::State *state){
        UNUSED(this);
        UNUSED(state);
        return false;
    });
    stateCheck->setLambda_Validity(stateValidityCheck);

    spaceInfo = std::make_shared<state_space::SpaceInformation>(m_Space);
    spaceInfo->setStateValidityCheck(stateCheck);

    mace::maps::OccupiedResult fillData = mace::maps::OccupiedResult::NOT_OCCUPIED;
    staticMap = new mace::maps::Data2DGrid<mace::maps::OccupiedResult>(&fillData, -43,43,-13,13,0.5,0.5);
    setupF3StaticMap();

    m_Planner = new mace::PotentialFields(spaceInfo,staticMap); //setup the potential fields planner

    m_castGoalState.setCoordinateFrame(CartesianFrameTypes::CF_LOCAL_NED);
    m_castGoalState.setXPosition(8); m_castGoalState.setYPosition(30);

    goalSpace = std::make_shared<mace::state_space::Cartesian2DSpace>();
    mace::state_space::Cartesian2DSpaceBounds bounds(-20,20,-10,10);
    goalSpace->setBounds(bounds);

    m_goalSampler = std::make_shared<mace::state_space::Cartesian2DSpace_Sampler>(goalSpace);

}


//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModulePathPlanningNASAPhase2::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;

//    std::shared_ptr<MaceCore::ModuleParameterStructure> vehicleParams = std::make_shared<MaceCore::ModuleParameterStructure>();
//    vehicleParams->AddTerminalParameters("VehicleID", MaceCore::ModuleParameterTerminalTypes::INT, true);
//    vehicleParams->AddTerminalParameters("Size", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
//    structure.AddNonTerminal("VehicleParameters", vehicleParams, true);

//    std::shared_ptr<MaceCore::ModuleParameterStructure> environmentParams = std::make_shared<MaceCore::ModuleParameterStructure>();
//    environmentParams->AddTerminalParameters("Vertices", MaceCore::ModuleParameterTerminalTypes::STRING, true);
//    structure.AddNonTerminal("EnvironmentParameters", environmentParams, true);

//    std::shared_ptr<MaceCore::ModuleParameterStructure> globalOrigin = std::make_shared<MaceCore::ModuleParameterStructure>();
//    globalOrigin->AddTerminalParameters("Latitude", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
//    globalOrigin->AddTerminalParameters("Longitude", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
//    structure.AddNonTerminal("GlobalOrigin", globalOrigin, true);

//    std::shared_ptr<MaceCore::ModuleParameterStructure> octomapParams = std::make_shared<MaceCore::ModuleParameterStructure>();
//    octomapParams->AddTerminalParameters("Filename", MaceCore::ModuleParameterTerminalTypes::STRING, false);
//    octomapParams->AddTerminalParameters("OctomapOperationalBoundary", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, false);
//    octomapParams->AddTerminalParameters("Resolution", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    octomapParams->AddTerminalParameters("Project2D", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, false);
//    octomapParams->AddTerminalParameters("MinRange", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    octomapParams->AddTerminalParameters("MaxRange", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    octomapParams->AddTerminalParameters("OccupancyThreshold", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    octomapParams->AddTerminalParameters("ProbabilityOfHit", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    octomapParams->AddTerminalParameters("ProbabilityOfMiss", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    octomapParams->AddTerminalParameters("MinThreshold", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    octomapParams->AddTerminalParameters("MaxThreshold", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    structure.AddNonTerminal("OctomapParameters", octomapParams, false);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


void ModulePathPlanningNASAPhase2::OnModulesStarted()
{
    MaceCore::ModuleBase::OnModulesStarted();

}

//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModulePathPlanningNASAPhase2::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    UNUSED(params);
}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModulePathPlanningNASAPhase2::AttachedAsModule(MaceCore::IModuleTopicEvents *ptr)
{
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
    ptr->Subscribe(this, m_MissionDataTopic.Name());

}

//!
//! \brief New non-spooled topic given
//!
//! NonSpooled topics send their data immediatly.
//! \param topicName Name of stopic
//! \param sender Module that sent topic
//! \param data Data for topic
//! \param target Target module (or broadcasted)
//!
void ModulePathPlanningNASAPhase2::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    UNUSED(topicName); UNUSED(sender); UNUSED(data); UNUSED(target);
}


//!
//! \brief New Spooled topic given
//!
//! Spooled topics are stored on the core's datafusion.
//! This method is used to notify other modules that there exists new data for the given components on the given module.
//! \param topicName Name of topic given
//! \param sender Module that sent topic
//! \param componentsUpdated Components in topic that where updated
//! \param target Target moudle (or broadcast)
//!
void ModulePathPlanningNASAPhase2::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    UNUSED(target);

    //example read of vehicle data
    if(topicName == m_VehicleDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), sender);

        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == mace::pose_topics::Topic_CartesianPosition::Name())
            {
                std::shared_ptr<mace::pose_topics::Topic_CartesianPosition> localPositionData = std::make_shared<mace::pose_topics::Topic_CartesianPosition>();
                m_VehicleDataTopic.GetComponent(localPositionData, read_topicDatagram);
                mace::pose::Abstract_CartesianPosition* currentPosition = localPositionData->getPositionObj();
                m_AgentPosition = *currentPosition->positionAs<mace::pose::CartesianPosition_3D>();
                this->updateAgentAction();
            }
            else if(componentsUpdated.at(i) == mace::pose_topics::Topic_CartesianVelocity::Name())
            {
                std::shared_ptr<mace::pose_topics::Topic_CartesianVelocity> localVelocityData = std::make_shared<mace::pose_topics::Topic_CartesianVelocity>();
                m_VehicleDataTopic.GetComponent(localVelocityData, read_topicDatagram);
                mace::pose::Velocity* currentVelocity = localVelocityData->getVelocityObj();
                m_AgentVelocity = *currentVelocity->velocityAs<mace::pose::Velocity_Cartesian3D>();
                this->updateAgentAction();
            }
            else if(componentsUpdated.at(i) == mace::pose_topics::Topic_AgentOrientation::Name())
            {
                mace::pose_topics::Topic_AgentOrientationPtr orientationData = std::make_shared<mace::pose_topics::Topic_AgentOrientation>();
                m_VehicleDataTopic.GetComponent(orientationData, read_topicDatagram);
                mace::pose::AbstractRotation* currentOrientation = orientationData->getRotationObj();
                m_AgentRotation = *currentOrientation->rotationAs<mace::pose::Rotation_3D>();
                this->updateAgentAction();
            }
        }
    }

    else if(topicName == m_MissionDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_MissionDataTopic.Name(), sender);

        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {

        }
    }
}

void ModulePathPlanningNASAPhase2::NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(vehicleID); UNUSED(sender);
}


void ModulePathPlanningNASAPhase2::NewlyLoadedOccupancyMap()
{

}

void ModulePathPlanningNASAPhase2::NewlyUpdatedOccupancyMap()
{

}

void ModulePathPlanningNASAPhase2::NewlyUpdatedGlobalOrigin(const GeodeticPosition_3D &position)
{
    UNUSED(position);
}

void ModulePathPlanningNASAPhase2::NewlyAvailableBoundary(const uint8_t &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(key); UNUSED(sender);
}


void ModulePathPlanningNASAPhase2::NewlyAvailableMission(const MissionItem::MissionList &mission)
{
    UNUSED(mission);
}

void ModulePathPlanningNASAPhase2::NewlyAvailableGoalState(const mace::state_space::GoalState &goal)
{
//    if(isOfType<mace::pose::Abstract_CartesianPosition*>(goal.getState()))
//    {
        this->m_goalState = goal;
        m_castGoalState = *m_goalState.getState()->stateAs<mace::pose::CartesianPosition_2D>();
        this->updateAgentAction();
//    }
}

void ModulePathPlanningNASAPhase2::cbiPlanner_SampledState(const mace::state_space::State *sampleState)
{
    UNUSED(sampleState);
}

void ModulePathPlanningNASAPhase2::cbiPlanner_NewConnection(const mace::state_space::State *beginState, const mace::state_space::State *secondState)
{
    UNUSED(beginState);
    UNUSED(secondState);
}

void ModulePathPlanningNASAPhase2::updateAgentAction()
{
    //VPF_ResultingForce artificialForce = computeVirtualForce(vehicleVelocity);
    //command_target::DynamicTarget newTarget = computeDynamicTarget(artificialForce, vehicleVelocity);
    command_target::DynamicTarget_Kinematic newTarget;

    double attractionGain = 0.5;
    double c1 = 1.5;
    double c2 = 0.5;

    double distance = m_castGoalState.distanceBetween2D(m_AgentPosition);
    double bearing = m_AgentPosition.polarBearingTo(&m_castGoalState);
    double deltaBearing = mace::math::angDistance(m_AgentRotation.getYaw(), bearing);

    std::cout<<"The distance to the target is: "<<mace::math::wrapTo2Pi(m_AgentRotation.getYaw())<<std::endl;
    std::cout<<"The current yaw is: "<<mace::math::wrapTo2Pi(m_AgentRotation.getYaw())<<std::endl;
    std::cout<<"The bearing to the target is: "<<bearing<<std::endl;
    std::cout<<"The delat to the target is: "<<deltaBearing<<std::endl;
    mace::pose::Rotation_2D rotationRate;
    rotationRate.setPhi(attractionGain * deltaBearing * (exp(-c1 * distance) + c2));
    newTarget.setYawRate(&rotationRate);

    mace::pose::Velocity_Cartesian3D currentVelocity;
    currentVelocity.setExplicitCoordinateFrame(CartesianFrameTypes::CF_BODY_NED);
    if(distance > 4)
        currentVelocity.setXVelocity(2.0);
    else
        currentVelocity.setXVelocity(2 * (distance / 4));

    currentVelocity.setYVelocity(0.0);
    currentVelocity.setZVelocity(0.0);
    newTarget.setVelocity(&currentVelocity);

//    command_item::Action_DynamicTarget newAction(255,1);
//    newAction.setDynamicTarget(newTarget);

//    ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr){
//        ptr->EventPP_ExecuteDynamicTarget(this, newAction);
//    });
}

VPF_ResultingForce ModulePathPlanningNASAPhase2::computeVirtualForce(double &vResponse)
{
    mace::pose::CartesianPosition_2D transformedPosition(m_AgentPosition);
    transformedPosition.setCoordinateFrame(CartesianFrameTypes::CF_LOCAL_ENU);
    transformedPosition.setXPosition(m_AgentPosition.getYPosition());
    transformedPosition.setYPosition(m_AgentPosition.getXPosition());

    mace::pose::Velocity_Cartesian2D transformedVelocity;
    transformedVelocity.setExplicitCoordinateFrame(CartesianFrameTypes::CF_LOCAL_ENU);
    transformedVelocity.setXVelocity(m_AgentVelocity.getYVelocity());
    transformedVelocity.setYVelocity(m_AgentVelocity.getXVelocity());

    VPF_ResultingForce resultingForce = m_Planner->computeArtificialForceVector(&transformedPosition, &transformedVelocity,
                                                                                &m_castGoalState, vResponse);
    return resultingForce;
}

command_target::DynamicTarget_Kinematic ModulePathPlanningNASAPhase2::computeDynamicTarget(const VPF_ResultingForce &apfObj, const double &vResponse)
{
//    double heading = wrapTo2Pi(atan2(apfObj.getForceY(), apfObj.getForceX()));
    double forceY = apfObj.getForceY();
    double forceX = apfObj.getForceX();
    double magnitude = sqrt(pow(forceY,2) + pow(forceX,2));
    double speedX = 0.0, speedY = 0.0;

    if(magnitude > vResponse)
    {
        speedX = vResponse * (forceX / magnitude);
        speedY = vResponse * (forceY / magnitude);
    }
    else {
        speedX = forceX;
        speedY = forceY;
    }

    command_target::DynamicTarget_Kinematic newTarget;
    mace::pose::Velocity_Cartesian3D newVelocity(CartesianFrameTypes::CF_LOCAL_NED);
    newVelocity.setXVelocity(speedY);
    newVelocity.setYVelocity(speedX);
    newVelocity.setZVelocity(0.0);
    newTarget.setVelocity(&newVelocity);

    myfile << speedY << ",";
    myfile << speedX << ",";
    myfile << m_AgentVelocity.getXVelocity() << ",";
    myfile << m_AgentVelocity.getYVelocity() << "\n";


//    mace::pose::Rotation_2D newHeading(polarToCompassBearing(heading));
//    newTarget.setYaw(&newHeading);

    return newTarget;
}

void ModulePathPlanningNASAPhase2::setupF3StaticMap()
{
    mace::maps::OccupiedResult* value;

    //construct the two horizontal portions
//    unsigned int xIndex = 0, yIndex = 0;
//    for(xIndex = 1; xIndex < staticMap->getSizeX() - 1; xIndex++)
//    {
//        value = staticMap->getCellByPosIndex(xIndex,yIndex);
//        *value = mace::maps::OccupiedResult::ENVIRONMENT_BOUNDARY;
//    }
//    yIndex = staticMap->getSizeY() - 1;
//    for(xIndex = 1; xIndex < staticMap->getSizeX() - 1; xIndex++)
//    {
//        value = staticMap->getCellByPosIndex(xIndex,yIndex);
//        *value = mace::maps::OccupiedResult::ENVIRONMENT_BOUNDARY;
//    }

//    //construct the two vertical portions
//    xIndex = 0;
//    for(yIndex = 0; yIndex < staticMap->getSizeY(); yIndex++)
//    {
//        value = staticMap->getCellByPosIndex(xIndex,yIndex);
//        *value = mace::maps::OccupiedResult::ENVIRONMENT_BOUNDARY;
//    }
//    xIndex = staticMap->getSizeX() - 1;
//    for(yIndex = 0; yIndex < staticMap->getSizeY(); yIndex++)
//    {
//        value = staticMap->getCellByPosIndex(xIndex,yIndex);
//        *value = mace::maps::OccupiedResult::ENVIRONMENT_BOUNDARY;
//    }

    //establish boundary around pillar one
    value = staticMap->getCellByPos(15,0);
    *value = mace::maps::OccupiedResult::OCCUPIED;

    //establish boundary around pillar two
    value = staticMap->getCellByPos(-15,0);
    *value = mace::maps::OccupiedResult::OCCUPIED;

    //establish boundary around pillar two
    value = staticMap->getCellByPos(-10,7);
    *value = mace::maps::OccupiedResult::OCCUPIED;

    //establish boundary around pillar two
    value = staticMap->getCellByPos(10,7);
    *value = mace::maps::OccupiedResult::OCCUPIED;

    //establish boundary around pillar two
    value = staticMap->getCellByPos(-2,9);
    *value = mace::maps::OccupiedResult::OCCUPIED;

    //establish boundary around pillar two
    value = staticMap->getCellByPos(4,7);
    *value = mace::maps::OccupiedResult::OCCUPIED;
}


