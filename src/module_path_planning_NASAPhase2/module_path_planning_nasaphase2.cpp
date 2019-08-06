#include "module_path_planning_nasaphase2.h"

#include "mace_core/module_factory.h"

#include <iostream>


ModulePathPlanningNASAPhase2::ModulePathPlanningNASAPhase2() :
    MaceCore::IModuleCommandPathPlanning(),
    m_VehicleDataTopic("vehicleData"),
    m_MissionDataTopic("vehicleMission"),
    m_PlanningStateTopic("planningState"),
    m_MapTopic("mappingData"),
    originSent(false),
    m_ProjectedOccupancyMap(nullptr),
    m_OccupiedVehicleMap(nullptr),
    m_OctomapSensorProperties(),
    m_Space(nullptr),
    sampler(nullptr),
    motionCheck(nullptr),
    stateCheck(nullptr),
    spaceInfo(nullptr),
    m_PlannerRRT(nullptr)
{    
    OccupiedResult fillValue = OccupiedResult::NOT_OCCUPIED;
    m_OccupiedVehicleMap = new maps::Data2DGrid<OccupiedResult>(&fillValue);

    m_Space = std::make_shared<state_space::Cartesian2DSpace>();

    sampler = std::make_shared<mace::state_space::Cartesian2DSpace_Sampler>(m_Space);
    motionCheck = std::make_shared<mace::state_space::DiscreteMotionValidityCheck>(m_Space);
    stateCheck = std::make_shared<mace::state_space::SpecialValidityCheck>(m_Space);


    auto stateValidityCheck = ([this](const mace::state_space::State *state){
        if(m_ProjectedOccupancyMap != nullptr)
        {
            const mace::pose::CartesianPosition_2D* castState = state->stateAs<const mace::pose::CartesianPosition_2D>();
            OccupiedResult* result = m_ProjectedOccupancyMap->getCellByPos(castState->getXPosition(),castState->getYPosition());
            if(*result == OccupiedResult::NOT_OCCUPIED)
                return true;
        }
        return false;
    });

    stateCheck->setLambda_Validity(stateValidityCheck);
    motionCheck->setStateValidityCheck(stateCheck);
    motionCheck->setMinCheckDistance(0.5);

    spaceInfo = std::make_shared<state_space::SpaceInformation>(m_Space);
    spaceInfo->setStateSampler(sampler);
    spaceInfo->setStateValidityCheck(stateCheck);
    spaceInfo->setMotionValidityCheck(motionCheck);

    m_PlannerRRT = std::make_shared<planners_sampling::RRTBase>(spaceInfo);
    m_PlannerRRT->setNearestNeighbor<nn::NearestNeighbor_FLANNLinear<planners_sampling::RootNode*>>();
    m_PlannerRRT->setCallbackFunction(this);
}


//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModulePathPlanningNASAPhase2::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;

    std::shared_ptr<MaceCore::ModuleParameterStructure> vehicleParams = std::make_shared<MaceCore::ModuleParameterStructure>();
    vehicleParams->AddTerminalParameters("VehicleID", MaceCore::ModuleParameterTerminalTypes::INT, true);
    vehicleParams->AddTerminalParameters("Size", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    structure.AddNonTerminal("VehicleParameters", vehicleParams, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> environmentParams = std::make_shared<MaceCore::ModuleParameterStructure>();
    environmentParams->AddTerminalParameters("Vertices", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    structure.AddNonTerminal("EnvironmentParameters", environmentParams, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> globalOrigin = std::make_shared<MaceCore::ModuleParameterStructure>();
    globalOrigin->AddTerminalParameters("Latitude", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    globalOrigin->AddTerminalParameters("Longitude", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    structure.AddNonTerminal("GlobalOrigin", globalOrigin, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> octomapParams = std::make_shared<MaceCore::ModuleParameterStructure>();
    octomapParams->AddTerminalParameters("Filename", MaceCore::ModuleParameterTerminalTypes::STRING, false);
    octomapParams->AddTerminalParameters("OctomapOperationalBoundary", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, false);
    octomapParams->AddTerminalParameters("Resolution", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("Project2D", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, false);
    octomapParams->AddTerminalParameters("MinRange", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("MaxRange", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("OccupancyThreshold", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("ProbabilityOfHit", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("ProbabilityOfMiss", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("MinThreshold", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    octomapParams->AddTerminalParameters("MaxThreshold", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
    structure.AddNonTerminal("OctomapParameters", octomapParams, false);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


void ModulePathPlanningNASAPhase2::OnModulesStarted()
{
    MaceCore::ModuleBase::OnModulesStarted();

    std::cout<<"All of the modules have been started."<<std::endl;
    ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr) {
        ptr->Event_SetGlobalOrigin(this, m_globalOrigin);
    });

    if(m_LocalOperationalBoundary.isValidPolygon() && !m_OctomapSensorProperties.isOctomapOperationalBoundary())
    {
        ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr) {
            BoundaryItem::BoundaryList boundary;
            BoundaryItem::BoundaryCharacterisic key({}, BoundaryItem::BOUNDARYTYPE::OPERATIONAL_FENCE);

            boundary.setBoundary(m_LocalOperationalBoundary);

            ptr->Event_SetBoundary(this, key, boundary);
        });
    }

    ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr) {
        ptr->EventPP_LoadOctomapProperties(this, m_OctomapSensorProperties);
    });


}

//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModulePathPlanningNASAPhase2::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    if(params->HasNonTerminal("VehicleParameters")) {
        std::shared_ptr<MaceCore::ModuleParameterValue> vehicleParamsXML = params->GetNonTerminalValue("VehicleParameters");
        localVehicleID = vehicleParamsXML->GetTerminalValue<int>("VehicleID");
        localVehicleSize = vehicleParamsXML->GetTerminalValue<double>("Size");
    }
    if(params->HasNonTerminal("GlobalOrigin")) {
        std::shared_ptr<MaceCore::ModuleParameterValue> globalOriginXML = params->GetNonTerminalValue("GlobalOrigin");
        double globalLat = globalOriginXML->GetTerminalValue<double>("Latitude");
        double globalLon = globalOriginXML->GetTerminalValue<double>("Longitude");

        m_globalOrigin = mace::pose::GeodeticPosition_3D(globalLat, globalLon, 0.0);

        // TODO: Figure out a way to send to the core (to fix github issue #126: )
/*        ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr) {
            ptr->Event_SetGlobalOrigin(this, m_globalOrigin);
        }); *///this one explicitly calls mace_core and its up to you to handle in core

        /*    ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
            }); */ //this is a general publication event, however, no one knows explicitly how to handle

    }
    if(params->HasNonTerminal("EnvironmentParameters")) {
        std::shared_ptr<MaceCore::ModuleParameterValue> environmentParams = params->GetNonTerminalValue("EnvironmentParameters");
        std::string vertsStr = environmentParams->GetTerminalValue<std::string>("Vertices");

        // Set up environment:
        mace::geometry::Polygon_2DG boundaryPolygon;
        parseBoundaryVertices(vertsStr, boundaryPolygon);

        if(boundaryPolygon.isValidPolygon())
        {
            if(!m_globalOrigin.isAnyPositionValid()) //if a global origin had not been assigned from above and we have a boundary let us choose one
            {
                m_globalOrigin.setLatitude(boundaryPolygon.at(0).getLatitude());
                m_globalOrigin.setLongitude(boundaryPolygon.at(0).getLongitude());
            }

            m_GlobalOperationalBoundary = boundaryPolygon;
            m_LocalOperationalBoundary.clearPolygon();
            for(size_t i = 0; i < boundaryPolygon.polygonSize(); i++)
            {
                mace::pose::GeodeticPosition_3D vertex(boundaryPolygon.at(i).getLatitude(),boundaryPolygon.at(i).getLongitude(),0.0);
                mace::pose::CartesianPosition_3D localVertex;

                mace::pose::DynamicsAid::GlobalPositionToLocal(&m_globalOrigin,&vertex,&localVertex);
                m_LocalOperationalBoundary.appendVertex(mace::pose::CartesianPosition_2D(localVertex.getXPosition(),localVertex.getYPosition()));
            }

            m_OccupiedVehicleMap->updateGridSize(m_LocalOperationalBoundary.getXMin(),m_LocalOperationalBoundary.getXMax(),
                                                 m_LocalOperationalBoundary.getYMin(),m_LocalOperationalBoundary.getYMax(),
                                                 m_OctomapSensorProperties.getTreeResolution(),m_OctomapSensorProperties.getTreeResolution());
        }

        if(params->HasNonTerminal("OctomapParameters")) {
            std::shared_ptr<MaceCore::ModuleParameterValue> octomapParams = params->GetNonTerminalValue("OctomapParameters");
            m_OctomapSensorProperties.setInitialLoadFile(octomapParams->GetTerminalValue<std::string>("Filename"));
            m_OctomapSensorProperties.setOctomapAsOperationalBoundary(octomapParams->GetTerminalValue<bool>("OctomapOperationalBoundary"));
            m_OctomapSensorProperties.setTreeResolution(octomapParams->GetTerminalValue<double>("Resolution"));
            m_OctomapSensorProperties.setMaxRange(octomapParams->GetTerminalValue<double>("MaxRange"));
            m_OctomapSensorProperties.setMinRange(octomapParams->GetTerminalValue<double>("MinRange"));
            m_OctomapSensorProperties.setOccupancyThreshold(octomapParams->GetTerminalValue<double>("OccupancyThreshold"));
            m_OctomapSensorProperties.setProbHit(octomapParams->GetTerminalValue<double>("ProbabilityOfHit"));
            m_OctomapSensorProperties.setProbMiss(octomapParams->GetTerminalValue<double>("ProbabilityOfMiss"));
            m_OctomapSensorProperties.setThreshMax(octomapParams->GetTerminalValue<double>("MaxThreshold"));
            m_OctomapSensorProperties.setThreshMin(octomapParams->GetTerminalValue<double>("MinThreshold"));
        }

    }

    else {
        throw std::runtime_error("Unkown Path Planning parameters encountered");
    }
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
    //example read of vehicle data
    if(topicName == m_VehicleDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), sender);

        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            //Ken Fix what should have been in here
        }
    }

    else if(topicName == m_MissionDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_MissionDataTopic.Name(), sender);

        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == MissionTopic::MissionItemReachedTopic::Name()) {
                std::cout<<"Path planner has seen that a mission item has been reached and will plan for the next one."<<std::endl;

                std::shared_ptr<MissionTopic::MissionItemReachedTopic> component = std::make_shared<MissionTopic::MissionItemReachedTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);
                //For now we will use this event to plan a route to the next valid mission item
                MissionItem::MissionKey key = component->getMissionKey();
                unsigned int indexAchieved = component->getMissionAchievedIndex();

                if((key == m_DynamicPlan.getAssociatedMissionKey()) && (indexAchieved == m_DynamicPlan.getAssociatedMissionItem()))
                {
                    //we have therefore completed the item that we had originally planned for
                    //let us generate a plan for the next one
                    indexAchieved++;
                    if(indexAchieved < m_MissionList.getQueueSize())
                    {
                        m_MissionList.setActiveIndex(indexAchieved);
                        replanRRT();
                    }
                    else
                    {
                        //there are no more items to currently plan for
                    }
                }
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionItemCurrentTopic::Name()) {
                std::cout<<"Path planner has seen that a mission item is the current item and should make sure this matches his formulated plan."<<std::endl;

                std::shared_ptr<MissionTopic::MissionItemCurrentTopic> component = std::make_shared<MissionTopic::MissionItemCurrentTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);

            }
        }
    }
}

void ModulePathPlanningNASAPhase2::NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(vehicleID);
}


void ModulePathPlanningNASAPhase2::NewlyLoadedOccupancyMap()
{
    //let us load the current static load octomap here so that if planning is required we have it ready to go
    m_ProjectedOccupancyMap = new maps::Data2DGrid<OccupiedResult>(this->getDataObject()->getCompressedOccupancyGrid2D());
    m_Space->setBounds(state_space::Cartesian2DSpaceBounds(m_ProjectedOccupancyMap->getXMin(), m_ProjectedOccupancyMap->getXMax(),
                                                           m_ProjectedOccupancyMap->getYMin(), m_ProjectedOccupancyMap->getYMax()));

    //this should probably be something that is driven via the core, however for now we will chcek operational boundary here
    if(m_OctomapSensorProperties.isOctomapOperationalBoundary()) //if the octomap is to be an operational boundary let us get the details for it
    {
        double minX, maxX, minY, maxY, minZ, maxZ;
        this->getDataObject()->getOctomapDimensions(minX, maxX, minY, maxY, minZ, maxZ);

        BoundaryItem::BoundaryList loadedBoundary;
        //Ken Fix This
//        loadedBoundary.appendVertexItem(mace::pose::CartesianPosition_2D(mace::CartesianFrameTypes::CF_LOCAL_ENU,minX,minY,"Lower Left"));
//        loadedBoundary.appendVertexItem(mace::pose::CartesianPosition_2D(mace::CartesianFrameTypes::CF_LOCAL_ENU,minX,maxY,"Upper Left"));
//        loadedBoundary.appendVertexItem(mace::pose::CartesianPosition_2D(mace::CartesianFrameTypes::CF_LOCAL_ENU,maxX,maxY,"Upper Right"));
//        loadedBoundary.appendVertexItem(mace::pose::CartesianPosition_2D(mace::CartesianFrameTypes::CF_LOCAL_ENU,maxX,minY,"Lower Right"));

        NewOperationalBoundary(loadedBoundary);

        ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr) {
            BoundaryItem::BoundaryList boundary;
            BoundaryItem::BoundaryCharacterisic key({}, BoundaryItem::BOUNDARYTYPE::OPERATIONAL_FENCE);

            boundary.setBoundary(m_LocalOperationalBoundary);

            ptr->Event_SetBoundary(this, key, boundary);
        });
    }
}

void ModulePathPlanningNASAPhase2::NewlyUpdatedOccupancyMap()
{
    //octomap::OcTree occupancyMap = this->getDataObject()->OccupancyMap_GetCopy();
    // Do something with occupancyMap

    //std::cout << "New grid from ROS module (in PP module): " << occupancyMap.size() << std::endl;
}

void ModulePathPlanningNASAPhase2::NewlyUpdatedGlobalOrigin(const GeodeticPosition_3D &position)
{
    //m_globalOrigin = std::make_shared<CommandItem::SpatialHome>(this->getDataObject()->GetGlobalOrigin());

    //std::cout << "New global origin received (PP): (" << m_globalOrigin->getPosition().getX() << " , " << m_globalOrigin->getPosition().getY() << ")" << std::endl;
}

void ModulePathPlanningNASAPhase2::NewlyAvailableBoundary(const uint8_t &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    BoundaryItem::BoundaryCharacterisic characterstic;
    this->getDataObject()->getCharactersticFromIdentifier(key, characterstic);
    if(characterstic.Type() == BoundaryItem::BOUNDARYTYPE::OPERATIONAL_FENCE)
    {
        BoundaryItem::BoundaryList boundary;
        this->getDataObject()->getBoundaryFromIdentifier(key, boundary);

        NewOperationalBoundary(boundary);
    }


    m_OccupiedVehicleMap->updateGridSize(m_LocalOperationalBoundary.getXMin(),m_LocalOperationalBoundary.getXMax(),
                                         m_LocalOperationalBoundary.getYMin(),m_LocalOperationalBoundary.getYMax(),
                                         m_OctomapSensorProperties.getTreeResolution(),m_OctomapSensorProperties.getTreeResolution());

    m_Space->setBounds(state_space::Cartesian2DSpaceBounds(m_LocalOperationalBoundary.getXMin(), m_LocalOperationalBoundary.getXMax(),
                                                           m_LocalOperationalBoundary.getYMin(), m_LocalOperationalBoundary.getYMax()));
}


void ModulePathPlanningNASAPhase2::NewlyAvailableMission(const MissionItem::MissionList &mission)
{
    if(mission.getMissionType() == MISSIONTYPE::GUIDED) //we should only be handling guided mission types, auto can go direct to vehicle or explored later
    {
        m_MissionList = mission;
        replanRRT();
        //at this point we would want to start preplanning the route based on beginning to ending nodes
    }
}

void ModulePathPlanningNASAPhase2::cbiPlanner_SampledState(const mace::state_space::State *sampleState)
{
    const mace::pose::CartesianPosition_2D* sampledState = sampleState->stateAs<mace::pose::CartesianPosition_2D>();
    std::shared_ptr<mace::pose_topics::Topic_CartesianPosition> ptrSampledState= std::make_shared<mace::pose_topics::Topic_CartesianPosition>();

    MaceCore::TopicDatagram topicDatagram;
    m_PlanningStateTopic.SetComponent(ptrSampledState, topicDatagram);
    ModulePathPlanningNASAPhase2::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_PlanningStateTopic.Name(), 0, MaceCore::TIME(), topicDatagram);
    });

}

void ModulePathPlanningNASAPhase2::cbiPlanner_NewConnection(const mace::state_space::State *beginState, const mace::state_space::State *secondState)
{
    mace::geometry::Line_Cartesian newLine;

    const mace::pose::CartesianPosition_2D* begState = beginState->stateAs<mace::pose::CartesianPosition_2D>();
    newLine.beginLine(begState);

    const mace::pose::CartesianPosition_2D* endState = secondState->stateAs<mace::pose::CartesianPosition_2D>();
    newLine.endLine(endState);

//    std::shared_ptr<mace::geometryTopic::Line_2DC_Topic> ptrLine= std::make_shared<mace::geometryTopic::Line_2DC_Topic>(newLine);

//    MaceCore::TopicDatagram topicDatagram;
//    m_PlanningStateTopic.SetComponent(ptrLine, topicDatagram);
//    ModulePathPlanningNASAPhase2::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
//        ptr->NewTopicDataValues(this, m_PlanningStateTopic.Name(), 0, MaceCore::TIME(), topicDatagram);
//    });

}

void ModulePathPlanningNASAPhase2::replanRRT()
{
    if(m_MissionList.getQueueSize() > 0)
    {

        mace::pose::CartesianPosition_2D goal2D;
        double altitude = 0.0;

        command_item::SpatialWaypoint* target;

        while(true) //this is dangerous, dont like it but for now let us just hack it in
        {
            unsigned int activeIndex = m_MissionList.getActiveIndex();

            if(activeIndex >= m_MissionList.getQueueSize()) //we are maxed out the ability and there are no valid points
                return;
            command_item::AbstractCommandItemPtr item = m_MissionList.getActiveMissionItem();
            target = item->as<command_item::SpatialWaypoint>();

            //Ken Fix This
//            if(target->getPosition().getCoordinateFrame() == Data::CoordinateFrameType::CF_LOCAL_ENU)
//            {
//                goal2D.setXPosition(target->getPosition().getX());
//                goal2D.setYPosition(target->getPosition().getY());
//                altitude = target->getPosition().getZ();
//            }

            if(stateCheck->isValid(&goal2D)) //we have found a valid goal, let us plan on this
                break;

            m_MissionList.setActiveIndex(activeIndex++);
        }

            mace::state_space::GoalState goalState(m_Space);
            goalState.setState(&goal2D);
            goalState.setRadialRegion(1.0);

            mace::pose::CartesianPosition_2D start(map_CurrentPosition[localVehicleID].getXPosition(), map_CurrentPosition[localVehicleID].getYPosition());
            mace::state_space::GoalState startState(m_Space);
            startState.setState(&start);

            m_PlannerRRT->setPlanningParameters(&startState,&goalState);

            std::vector<mace::state_space::State*> solution = m_PlannerRRT->solve();
            std::vector<mace::state_space::StatePtr> smartSolution;
            smartSolution.resize(solution.size());

            command_target::DynamicTargetList dynamicList;
            std::cout<<"The solution looks like this: "<<std::endl;

            for (int i = 0; i < solution.size(); i++)
            {
                mace::state_space::StatePtr state(solution[i]->getStateClone());

                CartesianPosition_3D pos;
                pos.setXPosition(state->stateAs<mace::pose::CartesianPosition_2D>()->getXPosition());
                pos.setYPosition(state->stateAs<mace::pose::CartesianPosition_2D>()->getYPosition());
                pos.setZPosition(altitude);
                command_target::DynamicTarget target;
                target.setPosition(&pos);

                dynamicList.appendDynamicTarget(target);

                smartSolution.at(i) = state;
                std::cout<<"X: "<<state->stateAs<mace::pose::CartesianPosition_2D>()->getXPosition()<<"Y: "<<state->stateAs<mace::pose::CartesianPosition_2D>()->getYPosition()<<std::endl;
            }

            m_DynamicPlan.setMissionKey(m_MissionList.getMissionKey());
            m_DynamicPlan.setAssociatedMissionItem(m_MissionList.getActiveIndex());
            m_DynamicPlan.setDynamicTargetList(dynamicList);

            ModulePathPlanningNASAPhase2::NotifyListeners([&](MaceCore::IModuleEventsPathPlanning* ptr) {
                ptr->EventPP_NewDynamicMissionQueue(this, m_DynamicPlan);
            });

    }
}
/**
 * @brief parseBoundaryVertices Given a string of delimited (lat, lon) pairs, parse into a vector of points
 * @param unparsedVertices String to parse with delimiters
 * @param globalOrigin Global position to convert relative to
 * @param vertices Container for boundary vertices
 */
void ModulePathPlanningNASAPhase2::parseBoundaryVertices(std::string unparsedVertices, mace::geometry::Polygon_2DG &boundaryPolygon)
{
    std::string nextVert;
    std::vector<std::string> verts;
    // For each character in the string
    for (std::string::const_iterator it = unparsedVertices.begin(); it != unparsedVertices.end(); it++) {
        // If we've hit the ';' terminal character
        if (*it == ';') {
            // If we have some characters accumulated
            if (!nextVert.empty()) {
                // Add them to the result vector
                verts.push_back(nextVert);
                nextVert.clear();
            }
        } else {
            // Accumulate the next character into the sequence
            nextVert += *it;
        }
    }

    // Now parse each string in the vector for each lat/lon to be inserted into our vertices vector:
    for(auto str : verts) {
        std::cout << "Vertex: " << str << std::endl;
        int pos = str.find_first_of(',');
        std::string lonStr = str.substr(pos+1);
        std::string latStr = str.substr(0, pos);
        double latitude = std::stod(latStr);
        double longitude = std::stod(lonStr);
        boundaryPolygon.appendVertex(mace::pose::GeodeticPosition_2D(latitude,longitude));
    }
}


//!
//! \brief A new Operational boundary has been provided, configure internal variables as needed
//! \param boundary New operational boundary
//!
void ModulePathPlanningNASAPhase2::NewOperationalBoundary(const BoundaryItem::BoundaryList &boundary)
{
    m_LocalOperationalBoundary.clearPolygon();
    m_LocalOperationalBoundary = boundary.boundingPolygon;

    if(m_globalOrigin.isAnyPositionValid()) //if a global origin had not been assigned from above and we have a boundary let us choose one
    {
        m_GlobalOperationalBoundary.clearPolygon();
        for(size_t i = 0; i < m_LocalOperationalBoundary.polygonSize(); i++)
        {
            CartesianPosition_3D vertex(m_LocalOperationalBoundary.at(i).getXPosition(),m_LocalOperationalBoundary.at(i).getYPosition(),0.0);
            GeodeticPosition_3D globalVertex;
            mace::pose::DynamicsAid::LocalPositionToGlobal(&m_globalOrigin,&vertex,&globalVertex);
            GeodeticPosition_2D globalVertex2D(globalVertex.getLatitude(),globalVertex.getLongitude());
            m_GlobalOperationalBoundary.appendVertex(globalVertex2D);
        }
    }

    m_OccupiedVehicleMap->updateGridSize(m_LocalOperationalBoundary.getXMin(),m_LocalOperationalBoundary.getXMax(),
                                         m_LocalOperationalBoundary.getYMin(),m_LocalOperationalBoundary.getYMax(),
                                         m_OctomapSensorProperties.getTreeResolution(),m_OctomapSensorProperties.getTreeResolution());

}

