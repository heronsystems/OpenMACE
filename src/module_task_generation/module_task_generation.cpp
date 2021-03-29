#include "module_task_generation.h"

ModuleTaskGeneration::ModuleTaskGeneration():
    MaceCore::IModuleCommandTaskGeneration (),
    m_VehicleDataTopic("vehicleData"),
    frontierBlobAlgorithm(false),
    frontierRoadAlgorithm(false),
    randomAlgorithm(false)
{

}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleTaskGeneration::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
}

//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleTaskGeneration::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;

    std::shared_ptr<MaceCore::ModuleParameterStructure> algorithm = std::make_shared<MaceCore::ModuleParameterStructure>();
    algorithm->AddTerminalParameters("Random", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, true);
    algorithm->AddTerminalParameters("Frontier_Blob", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, true);
    algorithm->AddTerminalParameters("Frontier_Road", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, true);
    structure.AddNonTerminal("AlgorithmParameters", algorithm, false);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleTaskGeneration::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    //printf("-------%x\n", this);
    //std::shared_ptr<frontierGeneration> f = std::make_shared<frontierGeneration>();
    //m_taskGenerationList.push_back(f);

    if(params->HasNonTerminal("AlgorithmParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> moduleSettings = params->GetNonTerminalValue("AlgorithmParameters");
        randomAlgorithm = moduleSettings->GetTerminalValue<bool>("Random");
        frontierBlobAlgorithm = moduleSettings->GetTerminalValue<bool>("Frontier_Blob");
        frontierRoadAlgorithm = moduleSettings->GetTerminalValue<bool>("Frontier_Road");

        if(frontierBlobAlgorithm)
        {
            std::cout<<"FOUND FRONTIER BLOB" <<endl;
        }

    }else
    {
        throw std::runtime_error("Unknown task generation parameters encountered");
    }


    //TODO add in road networks




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
void ModuleTaskGeneration::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
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
void ModuleTaskGeneration::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    //if vehicle hasn't been added yet then return. This can happen if a topic is sent before heartbeat is seen
    uint8_t vehicleID;
    if(this->getDataObject()->getMavlinkIDFromModule(sender, vehicleID)) {
        //example read of vehicle data
        if(topicName == m_VehicleDataTopic.Name())
        {
            //get latest datagram from mace_data
            MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), sender);
            //example of how to get data and parse through the components that were updated
            for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
                if(componentsUpdated.at(i) == DataStateTopic::StateAttitudeTopic::Name()) {
                    std::shared_ptr<DataStateTopic::StateAttitudeTopic> component = std::make_shared<DataStateTopic::StateAttitudeTopic>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                    // Update vehicle attitude data:
    //                updateAttitudeData(vehicleID, component);
                }
                else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                    std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> component = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                    // Update vehicle position data:
                    updateGlobalPositionData(vehicleID, component);
                }
            }
        }
    }
}


//!
//! \brief TestFunction
//! \param
//!
void ModuleTaskGeneration::TestFunction(const int &vehicleID)
{
    std::cout<< "TestFunction" <<std::endl;


    if(randomAlgorithm)
    {
        std::shared_ptr<TaskGeneration_RandomSpatial> random = std::make_shared<TaskGeneration_RandomSpatial>();
        random->assignMapLayerObject(this->getDataObject()->getLayeredMap().get());
        m_taskGenerationList.push_back(random);
    }
    if(frontierBlobAlgorithm)
    {
      // std::shared_ptr<frontierGeneration> f = std::make_shared<frontierGeneration>();

     //   frontierGeneration* frontier = new frontierGeneration();
        std::shared_ptr<frontierGeneration> f = std::make_shared<frontierGeneration>();
        f->assignMapLayerObject(this->getDataObject()->getLayeredMap().get());
        std::cout<<"Instantiated Frontier Algorithm" <<endl;
        //std::cout << (int)f->getAlgorithmType() << std::endl;
        printf("!!!!!!!!!!! %x\n", this);
        m_taskGenerationList.push_back(f);

        std::cout<<"m_taskGenerationList.size = " << m_taskGenerationList.size() <<endl;
    }

    mace::maps::OccupiedResult fillData = mace::maps::OccupiedResult::NOT_OCCUPIED;

    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* exampleMap = new mace::maps::Data2DGrid<mace::maps::OccupiedResult>(&fillData);
    exampleMap->updateGridSize(0,5,0,5,1,1);


    maps::OccupiedResult* value;

    for(int i =0; i<5; i++)
    {
        value= exampleMap->getCellByIndex(i);
        *value = maps::OccupiedResult::NOT_OCCUPIED;
    }

    value=exampleMap->getCellByIndex(5);
    *value=maps::OccupiedResult::OCCUPIED;

    value=exampleMap->getCellByIndex(6);
    *value=maps::OccupiedResult::OCCUPIED;

    value=exampleMap->getCellByIndex(7);
    *value=maps::OccupiedResult::UNKNOWN;

    for(int i =8; i<12; i++)
    {
        value= exampleMap->getCellByIndex(i);
        *value = maps::OccupiedResult::OCCUPIED;
    }

    value=exampleMap->getCellByIndex(12);
    *value=maps::OccupiedResult::NOT_OCCUPIED;

    for(int i =13; i<17; i++)
    {
        value= exampleMap->getCellByIndex(i);
        *value = maps::OccupiedResult::OCCUPIED;
    }

    value=exampleMap->getCellByIndex(17);
    *value=maps::OccupiedResult::UNKNOWN;

    value=exampleMap->getCellByIndex(18);
    *value=maps::OccupiedResult::OCCUPIED;

    value=exampleMap->getCellByIndex(19);
    *value=maps::OccupiedResult::OCCUPIED;

    for(int i =20; i<25; i++)
    {
        value= exampleMap->getCellByIndex(i);
        *value = maps::OccupiedResult::NOT_OCCUPIED;
    }

//    std::unordered_map<std::string, mace::maps::BaseGridMap*> map;
//    std::string example = "example";
//    maps::LayeredMap* layeredMap = new maps::LayeredMap(map);

//    map.insert(std::make_pair(example, exampleMap));

    std::string layerName = "local_OccupancyLayer";
    this->getDataObject()->updateLayeredMapLayer(layerName, exampleMap);

    // Vehicle pose from pose map:
    for(auto vehicle : m_currentVehiclePoseMap) {
        int vehicleID = vehicle.first;
        mace::maps::CartesianPosition_2D* vehiclePose = vehicle.second;
    // Test vehicle pose:
//        mace::maps::CartesianPosition_2D* vehiclePose = new mace::maps::CartesianPosition_2D();
//        vehiclePose->setXPosition(2);
//        vehiclePose->setYPosition(2);

        // Generate task list based on vehicle positions:
        std::list<std::shared_ptr<TaskDescriptor>> taskList;

        for(int i = 0; i< m_taskGenerationList.size(); i++)
        {
            if(m_taskGenerationList.at(i)->getAlgorithmType() == Abstract_TaskGeneration::AlgorithmTypes::FRONTIER)
            {
                std::static_pointer_cast<frontierGeneration>(m_taskGenerationList.at(i))->newlyUpdatedMapLayer(layerName, vehiclePose);
                taskList.merge(std::static_pointer_cast<frontierGeneration>(m_taskGenerationList.at(i))->generateFrontierWaypoint());
            }
            if(m_taskGenerationList.at(i)->getAlgorithmType() == Abstract_TaskGeneration::AlgorithmTypes::RANDOM)
            {
                std::static_pointer_cast<TaskGeneration_RandomSpatial>(m_taskGenerationList.at(i))->newlyUpdatedMapLayer(layerName, vehiclePose);
                taskList.merge(std::static_pointer_cast<TaskGeneration_RandomSpatial>(m_taskGenerationList.at(i))->generateRandomWaypoint());
            }
//            // TODO: Add Road network
//            if(m_taskGenerationList.at(i)->getAlgorithmType == Abstract_TaskGeneration::AlgorithmTypes::FRONTIER_ROAD_NETWORK)
//            {
//            }
        }

        ModuleTaskGeneration::NotifyListeners([&](MaceCore::IModuleEventsTaskGeneration* ptr) {
            ptr->TaskGeneration_NewTaskList(this, taskList);
        });
    }

}

//!
//! \brief updateGlobalPositionData Update the position of the corresponding vehicle and convert to a local position (from Geodetic 3D)
//! \param vehicleID ID of the vehicle to update
//! \param component Position (in a global, Geodetic frame)
//!
void ModuleTaskGeneration::updateGlobalPositionData(const int &vehicleID, const std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> &component)
{
    //the command here is based in a global cartesian space, we therefore need to transform it
    CartesianPosition_3D cartesianPosition;
    GeodeticPosition_3D globalOrigin = this->getDataObject()->GetGlobalOrigin();
    GeodeticPosition_3D currentPosition(component->getLatitude(),
                                        component->getLongitude(),
                                        component->getAltitude());

    DynamicsAid::GlobalPositionToLocal(globalOrigin, currentPosition, cartesianPosition);

    double northing = cartesianPosition.getYPosition();
    double easting = cartesianPosition.getXPosition();
    double altitude = cartesianPosition.getZPosition();

    std::vector<int> connectedVehicles;
    this->getDataObject()->GetLocalVehicles(connectedVehicles);
//    if(std::find(connectedVehicles.begin(), connectedVehicles.end(), vehicleID) != connectedVehicles.end()) {
    for(auto id : connectedVehicles) {
        if(id == vehicleID) {
            DataState::StateLocalPosition tmpPos = DataState::StateLocalPosition(component->getCoordinateFrame(), northing, easting, altitude);

            if(m_currentVehiclePoseMap.find(id) != m_currentVehiclePoseMap.end()) {
                m_currentVehiclePoseMap.at(id)->setXPosition(tmpPos.getPositionX());
                m_currentVehiclePoseMap.at(id)->setYPosition(tmpPos.getPositionY());
            }
            else {
                mace::pose::CartesianPosition_2D* pos = new mace::pose::CartesianPosition_2D(tmpPos.getPositionX(), tmpPos.getPositionY());
                m_currentVehiclePoseMap.insert(std::make_pair(id, pos));
            }

//            MaceLog::Alert("Position updated for vehicle ID: " + std::to_string(vehicleID));
        }
    }
}

void ModuleTaskGeneration::LayeredMapLayerUpdated(const std::string &layerName) {
    // Grab pointer from core for the layer name...
//    const shared_ptr<mace::maps::LayeredMap> tmp = this->getDataObject()->getLayeredMap();

//    mace::maps::CartesianPosition_2D* vehiclePose = new mace::maps::CartesianPosition_2D();

//    vehiclePose->setXPosition(2);
//    vehiclePose->setYPosition(2);


//    std::list<std::shared_ptr<TaskDescriptor>> taskList;
//    for(int i = 0; i< m_taskGenerationList.size(); i++)
//    {
//        if(m_taskGenerationList.at(i)->getAlgorithmType() == Abstract_TaskGeneration::AlgorithmTypes::FRONTIER)
//        {
//            for(auto vehicle : m_currentVehiclePoseMap) {
//                std::static_pointer_cast<frontierGeneration>(m_taskGenerationList.at(i))->newlyUpdatedMapLayer(layerName, vehiclePose);
//                taskList.merge(std::static_pointer_cast<frontierGeneration>(m_taskGenerationList.at(i))->generateFrontierWaypoint());
//            }

//        }
//        if(m_taskGenerationList.at(i)->getAlgorithmType() == Abstract_TaskGeneration::AlgorithmTypes::RANDOM)
//        {
//            std::static_pointer_cast<TaskGeneration_RandomSpatial>(m_taskGenerationList.at(i))->newlyUpdatedMapLayer(layerName, vehiclePose);
//            taskList.merge(std::static_pointer_cast<TaskGeneration_RandomSpatial>(m_taskGenerationList.at(i))->generateRandomWaypoint());
//        }
//    }

//    ModuleTaskGeneration::NotifyListeners([&](MaceCore::IModuleEventsTaskGeneration* ptr) {
//        ptr->TaskGeneration_NewTaskList(this, taskList);
//    });
}


//!
//! \brief NewlyAvailableVehicle Subscriber to a newly available vehilce topic
//! \param vehicleID Vehilce ID of the newly available vehicle
//!
void ModuleTaskGeneration::NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(vehicleID);
    UNUSED(sender);
}


