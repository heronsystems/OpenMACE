#include "module_rta.h"

#include <memory>

#include <iostream>

//!
//! \brief ModuleRTA Default constructor
//!
ModuleRTA::ModuleRTA():
    m_gridSpacing(1),
    m_globalInstance(true),
    gridSpacingSent(false),
    environmentBoundarySent(false),
    m_VehicleDataTopic("vehicleData"),
    m_SensorDataTopic("sensorData"),
    m_SensorFootprintDataTopic("sensorFootprint")
{
    std::vector<CartesianPosition_2D> localBoundaryVerts;
    Polygon_Cartesian poly(localBoundaryVerts);

    environment = std::make_shared<Environment_Map>(poly, m_gridSpacing, m_globalInstance);
}

ModuleRTA::~ModuleRTA(){

}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleRTA::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
    ptr->Subscribe(this, m_SensorFootprintDataTopic.Name());
}


//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleRTA::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;

    std::shared_ptr<MaceCore::ModuleParameterStructure> moduleSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    moduleSettings->AddTerminalParameters("GlobalInstance", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, true);
    moduleSettings->AddTerminalParameters("SpecalizationID", MaceCore::ModuleParameterTerminalTypes::INT, false);
    structure.AddNonTerminal("ModuleParameters", moduleSettings, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> environmentParams = std::make_shared<MaceCore::ModuleParameterStructure>();
    environmentParams->AddTerminalParameters("GridSpacing", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    structure.AddNonTerminal("EnvironmentParameters", environmentParams, true);

    structure.AddTerminalParameters("ID", MaceCore::ModuleParameterTerminalTypes::INT, false);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleRTA::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    MaceCore::Metadata_RTA meta;

    if(params->HasTerminal("ID"))
    {
        this->SetID(params->GetTerminalValue<int>("ID"));
    }

    if(params->HasNonTerminal("ModuleParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> moduleSettings = params->GetNonTerminalValue("ModuleParameters");
        m_globalInstance = moduleSettings->GetTerminalValue<bool>("GlobalInstance");

        if(m_globalInstance == true)
        {
            meta.SetGlobal();
        }
        else
        {
            if(moduleSettings->HasTerminal("SpecalizationID") == false)
            {
                throw std::runtime_error ("RTA module has been marked as not-global, yet no SpecalizationID parameter given");
            }
            int specializationID = moduleSettings->GetTerminalValue<int>("SpecalizationID");
            meta.SetSpecalization(specializationID);
        }
    }

    if(params->HasNonTerminal("EnvironmentParameters")) {
        std::shared_ptr<MaceCore::ModuleParameterValue> environmentParams = params->GetNonTerminalValue("EnvironmentParameters");
        m_gridSpacing = environmentParams->GetTerminalValue<double>("GridSpacing");

        //        // Set grid spacing in MACE:
        //        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
        //            ptr->Event_SetGridSpacing(this, gridSpacing);
        //        });
    }
    else {
        throw std::runtime_error("Unkown RTA parameters encountered");
    }

    this->setModuleMetaData(meta);
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
void ModuleRTA::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
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
void ModuleRTA::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    UNUSED(target);

//    if(!gridSpacingSent) {
//        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
//            ptr->Event_SetGridSpacing(this, m_gridSpacing);
//        });

//        gridSpacingSent = true;
//    }


    //example read of vehicle data
    if(topicName == m_VehicleDataTopic.Name())
    {
        uint8_t senderID;
        if(this->getDataObject()->getMavlinkIDFromModule(sender, senderID)) {
            MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), sender);
            for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {

                if(componentsUpdated.at(i) == mace::pose_topics::Topic_CartesianPosition::Name())
                {
                    std::shared_ptr<mace::pose_topics::Topic_CartesianPosition> localPositionData = std::make_shared<mace::pose_topics::Topic_CartesianPosition>();
                    m_VehicleDataTopic.GetComponent(localPositionData, read_topicDatagram);
                    CartesianPosition_2D vehiclePosition;
//                    vehiclePosition.setXPosition(localPositionData.get()->getX());
//                    vehiclePosition.setYPosition(localPositionData.get()->getY());
//                    m_vehicles[senderID] = vehiclePosition;
                }
                else if(componentsUpdated.at(i) == mace::pose_topics::Topic_GeodeticPosition::Name()) {
                    std::shared_ptr<mace::pose_topics::Topic_GeodeticPosition> globalPositionData = std::make_shared<mace::pose_topics::Topic_GeodeticPosition>();
                    m_VehicleDataTopic.GetComponent(globalPositionData, read_topicDatagram);

//                    // Update vehicle position:
//                    Position<CartesianPosition_2D> tmpPos;
//                    DataState::StateLocalPosition localPositionData;
//                    DataState::StateGlobalPosition tmpGlobalOrigin;
//                    mace::pose::GeodeticPosition_3D origin = this->getDataObject()->GetGlobalOrigin();
//                    command_item::SpatialHome tmpSpatialHome(origin);

//                    tmpGlobalOrigin.setLatitude(tmpSpatialHome.getPosition().getX());
//                    tmpGlobalOrigin.setLongitude(tmpSpatialHome.getPosition().getY());
//                    tmpGlobalOrigin.setAltitude(tmpSpatialHome.getPosition().getZ());

//                    DataState::PositionalAid::GlobalPositionToLocal(tmpGlobalOrigin, *globalPositionData, localPositionData);
//                    tmpPos.setXPosition(localPositionData.getX());
//                    tmpPos.setYPosition(localPositionData.getY());
//                    // Insert/update into map
//                    m_vehicles[senderID] = tmpPos;

                }
            }
        }

    }else if(topicName == m_SensorFootprintDataTopic.Name())
    {
//        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_SensorFootprintDataTopic.Name(), senderID);
//        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
//            if(componentsUpdated.at(i) == DataVehicleSensors::SensorVertices_Global::Name()) {
//                std::shared_ptr<DataVehicleSensors::SensorVertices_Global> sensorVerticesGlobal = std::make_shared<DataVehicleSensors::SensorVertices_Global>();
//                m_SensorFootprintDataTopic.GetComponent(sensorVerticesGlobal, read_topicDatagram);
//            }
//        }
    }
}

/**
 * @brief updateEnvironment Given a new boundary, update the environment and Voronoi partitions
 * @param boundary New boundary to partition/generate targets for
 */
void ModuleRTA::updateEnvironment(const BoundaryItem::BoundaryList &boundary)
{
    std::cout<<"Update environment (RTA)."<<std::endl;

    Polygon_Cartesian poly = boundary.boundingPolygon;
//    m_gridSpacing = this->getDataObject()->GetGridSpacing();

    environment = std::make_shared<Environment_Map>(poly, m_gridSpacing, m_globalInstance);

    m_globalOrigin = std::make_shared<command_item::SpatialHome>();
    mace::pose::GeodeticPosition_3D currentOrigin = this->getDataObject()->GetGlobalOrigin();
    m_globalOrigin->setPosition(&currentOrigin);
    //  2) Re-partition space
    environment->computeBalancedVoronoi(m_vehicles);
    m_vehicleCells.clear();
    m_vehicleCells = environment->getCells();
}

//!
//! \brief NewlyUpdatedGridSpacing Grid spacing subscriber to update nodes within the environment
//!
void ModuleRTA::NewlyUpdatedGridSpacing() {
    //There is no way this can happen on the local instance
    //updateEnvironment();

    //    if(m_globalInstance) {
    //        //      b) Publish topic to core with new boundary data
    //        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
    //            ptr->Event_SetVehicleBoundaryVertices(this, m_vehicleCells);
    //        });
    //    }
}

//!
//! \brief NewlyUpdatedGlobalOrigin Subsciber for a new global origin position
//! \param position Geodetic 3D position
//!
void ModuleRTA::NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &position) {
    UNUSED(position);
    //I dont think we have to do this here
    //updateEnvironment();

    //    if(m_globalInstance) {
    //        //      b) Publish topic to core with new boundary data
    //        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
    //            ptr->Event_SetVehicleBoundaryVertices(this, m_vehicleCells);
    //        });
    //    }
}

//! \brief NewlyUpdatedBoundaryVertices Function partitioning the space using the voronoi
//! decomposition. The result of the function should be another boundary list notifying external
//! agents of their appropriately newly assigned resource fence.
//! \param boundary obj defining the operational fence as defined by an external party. This
//! will be the space that is actually partitioned into voronoi cells.
//!
void ModuleRTA::NewlyAvailableBoundary(const uint8_t &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);

    BoundaryItem::BoundaryCharacterisic characterstic;
    BoundaryItem::BoundaryList boundary;
    this->getDataObject()->getBoundaryFromIdentifier(key, boundary);
    this->getDataObject()->getCharactersticFromIdentifier(key, characterstic);

    /// If its not global and is a resource fence
    ///   Check if we care about module and do further processing
    if(this->getModuleMetaData().IsGlobal() == false && characterstic.Type() == BoundaryItem::BOUNDARYTYPE::RESOURCE_FENCE)
    {
        // Determine the module which this RTA module is a specalization of
        MaceCore::ModuleCharacteristic specalizedModule;
        specalizedModule.MaceInstance = this->getParentMaceInstanceID();
        specalizedModule.ModuleID = this->getModuleMetaData().SpecalizationOf();

        // Determine the vehicleID this RTA module is a specalization of
//        int vehicleID = this->getDataObject()->getMavlinkIDFromModule(specalizedModule);
        uint8_t vehicleID;
        if(this->getDataObject()->getMavlinkIDFromModule(specalizedModule, vehicleID)) {
            // Determine the module that the given boundary is assositated with
            if(characterstic.ContainsVehicle(vehicleID) == false) return;

            printf("RTA Module for Vehicle %d received a new Rsource Fence\n", vehicleID);
        }


        // /////////////////////////////
        // PROCESS BOUNDARY
        // /////////////////////////////
    }


    /// If this module is a global instance and the boundary is an operational fence:
    ///   Then partition out and give new resource boundaries to Core.
    if(this->getModuleMetaData().IsGlobal() == true && characterstic.Type() == BoundaryItem::BOUNDARYTYPE::OPERATIONAL_FENCE) {

        //MTB - According to Pat, this method only makes sense to call on global RTA module that is partioning a OP-fence to resource-fence
        updateEnvironment(boundary);

        for(auto vehicleCell : m_vehicleCells) {
            int vehicleID = vehicleCell.first;
            BoundaryItem::BoundaryList resourceFence;
            mace::geometry::Polygon_Cartesian polyBoundary;
            for(auto vertex : vehicleCell.second.getVector()) {
                polyBoundary.appendVertex(vertex);
            }
            resourceFence.setBoundary(polyBoundary);

            const BoundaryItem::BoundaryCharacterisic key({vehicleID}, BoundaryItem::BOUNDARYTYPE::RESOURCE_FENCE);

            ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr) {
                ptr->Event_SetBoundary(this, key, resourceFence);
            });
        }

    }

}


//!
//! \brief NewlyAvailableVehicle Subscriber for a new vehicle topic
//! \param vehicleID Vehicle ID of the new vehicle
//!
void ModuleRTA::NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(vehicleID);
    UNUSED(sender);

//    std::vector<MaceCore::TopicCharacteristic> x = this->GetEmittedTopics();

//    MaceCore::TopicDatagram topicDatagram;
//    for(size_t i = 0 ; i < components.size() ; i++)
//    {
//        m_TaskTopic.SetComponent(components.at(i), topicDatagram);
//        //notify listeners of topic
//        NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
//            ptr->NewTopicDataValues(this, m_TaskTopic.Name(), vehicleID, MaceCore::TIME(), topicDatagram);
//        });
//    }



    /* MTB - Removing 7/2/2018
     * @pnolan Issue: 137
     *
     * NewlyAvailableVehicle my get called before the first vehicleData topic gets transmitted
     * Should split functionality between this method and ModuleRTA::NewTopicSpooled
     *
     * This code currently performs no side-effects, so I am removing
     *
    MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), vehicleID);
    std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> globalPositionData = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
    m_VehicleDataTopic.GetComponent(globalPositionData, read_topicDatagram);

    // Set vehicle and compute Voronoi:
    CommandItem::SpatialHome globalOrigin = this->getDataObject()->GetGlobalOrigin();
    if(globalOrigin.getPosition().has2DPositionSet()) {
        DataState::StateLocalPosition localPositionData;
        DataState::StateGlobalPosition tmpGlobalOrigin;
        tmpGlobalOrigin.setLatitude(globalOrigin.getPosition().getX());
        tmpGlobalOrigin.setLongitude(globalOrigin.getPosition().getY());
        tmpGlobalOrigin.setAltitude(globalOrigin.getPosition().getZ());

        DataState::StateGlobalPosition tmpGlobalPosition;
        tmpGlobalPosition.setLatitude(globalPositionData->getLatitude());
        tmpGlobalPosition.setLongitude(globalPositionData->getLongitude());
        tmpGlobalPosition.setAltitude(globalPositionData->getAltitude());

        DataState::PositionalAid::GlobalPositionToLocal(tmpGlobalOrigin, tmpGlobalPosition, localPositionData);

        //        Point localPosition(localPositionData.getX(), localPositionData.getY(), localPositionData.getZ());
        //        bool updateMaceCore = environment->updateVehiclePosition(vehicleID, localPosition, true); // True for recomputing voronoi, false for adding to the vehicle map
        //        if(updateMaceCore){
        ////            updateMACEMissions(environment->getCells());
        //        }
    }
    else {
        std::cout << "No global origin set. Cannot update missions for MACE" << std::endl;
        return;
    }
    */
}

/**
 * @brief updateMACEMissions Sends new missions to MACE for each vehicle in the provided list
 * @param updateCells Map of cells that contain node lists to send to MACE
 * @param direction Grid direction for missions (NORTH_SOUTH, EAST_WEST, or CLOSEST_POINT)
 */
void ModuleRTA::updateMACEMissions(std::map<int, Cell_2DC> updateCells, GridDirection direction) {
    UNUSED(updateCells);
    UNUSED(direction);

    //    DataState::StateGlobalPosition tmpGlobalOrigin;
    //    if(environment->getGlobalOrigin()->has2DPositionSet()) {
    //        tmpGlobalOrigin.setLatitude(environment->getGlobalOrigin()->getLatitude());
    //        tmpGlobalOrigin.setLongitude(environment->getGlobalOrigin()->getLongitude());
    //        tmpGlobalOrigin.setAltitude(environment->getGlobalOrigin()->getAltitude());
    //    }
    //    else {
    //        std::cout << "No global origin set. Cannot update missions for MACE" << std::endl;
    //        return;
    //    }

    //    // For every cell, send to MACE its node list:
    //    if(tmpGlobalOrigin.has2DPositionSet()) {
    //        if(updateCells.size() > 0) {
    //            for(auto cell : updateCells) {
    //                int vehicleID = cell.first;

    //                MissionItem::MissionList missionList;
    //                missionList.setMissionTXState(MissionItem::MISSIONSTATE::PROPOSED);
    //                missionList.setMissionType(MissionItem::MISSIONTYPE::AUTO);
    //                missionList.setVehicleID(vehicleID);

    //                // Grab the sorted points from the cell:
    //                // Loop over sorted points and insert into a mission:
    //                for(auto point : cell.second.getNodes()) {
    //                    std::shared_ptr<CommandItem::SpatialWaypoint> newWP = std::make_shared<CommandItem::SpatialWaypoint>();
    //                    newWP->setTargetSystem(vehicleID);

    //                    DataState::StateLocalPosition tmpLocalPoint;
    //                    tmpLocalPoint.setX(point->getXPosition());
    //                    tmpLocalPoint.setY(point->getYPosition());
    //                    tmpLocalPoint.setZ(10);

    //                    DataState::StateGlobalPosition tmpGlobalPoint;
    //                    DataState::PositionalAid::LocalPositionToGlobal(tmpGlobalOrigin, tmpLocalPoint, tmpGlobalPoint);
    //                    newWP->setPosition(tmpGlobalPoint);

    //                    missionList.insertMissionItem(newWP);
    //                }

    //                ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr){
    //                    ptr->GSEvent_UploadMission(this, missionList);
    //                });
    //            }
    //        }
    //        else {
    //            std::cout << "No cells in environment to update." << std::endl;
    //        }
    //    }
    //    else {
    //        std::cout << "No global origin set. Cannot update MACE missions via RTA." << std::endl;
    //    }
}


void ModuleRTA::TestFunction(const int &vehicleID)
{
    UNUSED(vehicleID);

    //    GridDirection direction = GridDirection::EAST_WEST;
    //    std::map<int, Position<CartesianPosition_2D> > vehicles;
    //    Position<CartesianPosition_2D> pt1, pt2;
    //    pt1.setXPosition(5); pt1.setYPosition(5);
    //    pt2.setXPosition(-5); pt2.setYPosition(-5);
    //    vehicles.insert(std::make_pair(1, pt1));
    //    vehicles.insert(std::make_pair(2, pt2));

    //    bool updateMaceCore = environment->computeBalancedVoronoi(vehicles);
    //    if(updateMaceCore) {
    //        updateMACEMissions(environment->getCells(), direction);
    //    }

    //    std::map<int, Cell_2DC> cells = environment->getCells();
    //    for(auto cell : cells) {
    //        environment->printCellInfo(cell.second);
    //    }

//    DataState::StateGlobalPosition tmpGlobalOrigin;
//    if(environment->getGlobalOrigin()->getPosition().has2DPositionSet()) {
//        tmpGlobalOrigin.setLatitude(environment->getGlobalOrigin()->getPosition().getX());
//        tmpGlobalOrigin.setLongitude(environment->getGlobalOrigin()->getPosition().getY());
//        tmpGlobalOrigin.setAltitude(environment->getGlobalOrigin()->getPosition().getZ());
//    }
//    else {
//        std::cout << "No global origin set. Cannot update missions for MACE" << std::endl;
//        return;
//    }


//    std::vector<Position<CartesianPosition_2D> > pts;
//    Position<CartesianPosition_2D> pt1; pt1.setXPosition(0.0); pt1.setYPosition(0.0);
//    Position<CartesianPosition_2D> pt2; pt2.setXPosition(10.0); pt2.setYPosition(0.0);
//    Position<CartesianPosition_2D> pt3; pt3.setXPosition(30.0); pt3.setYPosition(0.0);
//    Position<CartesianPosition_2D> pt4; pt4.setXPosition(80.0); pt4.setYPosition(0.0);
//    pts.push_back(pt1); pts.push_back(pt2); pts.push_back(pt3); pts.push_back(pt4);

//    MissionItem::MissionList missionList;
//    missionList.setMissionTXState(MissionItem::MISSIONSTATE::PROPOSED);
//    missionList.setMissionType(MissionItem::MISSIONTYPE::AUTO);
//    missionList.setVehicleID(2);

//    for(auto point : pts) {
//        std::shared_ptr<CommandItem::SpatialWaypoint> newWP = std::make_shared<CommandItem::SpatialWaypoint>();
//        newWP->setTargetSystem(vehicleID);

//        DataState::StateLocalPosition tmpLocalPoint;
//        tmpLocalPoint.setX(point.getXPosition());
//        tmpLocalPoint.setY(point.getYPosition());
//        tmpLocalPoint.setZ(10);

//        DataState::StateGlobalPosition tmpGlobalPoint;
//        DataState::PositionalAid::LocalPositionToGlobal(tmpGlobalOrigin, tmpLocalPoint, tmpGlobalPoint);
//        newWP->setPosition(tmpGlobalPoint);

//        missionList.insertMissionItem(newWP);
//    }

//        ModuleRTA::NotifyListeners([&](MaceCore::IModuleEventsRTA* ptr){
    //        ptr->GSEvent_UploadMission(this, missionList);
    //    });

}
