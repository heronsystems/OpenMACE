#include "module_vehicle_sensors.h"

ModuleVehicleSensors::ModuleVehicleSensors():
    m_truthBTFile(""),
    m_VehicleDataTopic("vehicleData"), m_SensorDataTopic("sensorData"), m_SensorFootprintDataTopic("sensorFootprint")
{
//    cameraSensor = new DataVehicleSensors::SensorCamera();
    m_circularCameraSensor = std::make_shared<DataVehicleSensors::SensorCircularCamera>();

//    // TESTING:
//    double minX = -50.0; double maxX = 50.0;
//    double minY = -50.0; double maxY = 50.0;
//    double x_res = 10.0; double y_res = 10.0;
//    mace::maps::MapCell truthFill(mace::maps::OccupiedResult::NOT_OCCUPIED, 1.0, false);
//    mace::maps::MapCell localFill(mace::maps::OccupiedResult::UNKNOWN, 0.0, false);
//    // TODO: Load this from config file?
////    mace::maps::OccupiedResult truthFill = mace::maps::OccupiedResult::NOT_OCCUPIED;
////    mace::maps::OccupiedResult localFill = mace::maps::OccupiedResult::UNKNOWN;
//    m_compressedMapTruth = new mace::maps::Data2DGrid<mace::maps::MapCell>(&truthFill, minX, maxX, minY, maxY, x_res, y_res);
//    mace::maps::Data2DGrid<mace::maps::MapCell>* localMap = new mace::maps::Data2DGrid<mace::maps::MapCell>(&localFill, minX, maxX, minY, maxY, x_res, y_res);
//    // END TESTING (todo: initialize maps when not testing)

//    m_LayeredMap_Truth = std::make_shared<mace::maps::LayeredMap>();
//    m_LayeredMap_Truth->updateMapLayer("truth_OccupancyLayer", m_compressedMapTruth);
//    this->getDataObject()->updateLayeredMapLayer("local_OccupancyLayer", localMap);
}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleVehicleSensors::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
}

//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleVehicleSensors::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;
//    std::shared_ptr<MaceCore::ModuleParameterStructure> cameraSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
//    cameraSettings->AddTerminalParameters("CameraName", MaceCore::ModuleParameterTerminalTypes::STRING, true);
//    cameraSettings->AddTerminalParameters("FocalLength", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
//    cameraSettings->AddTerminalParameters("SensorWidth", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
//    cameraSettings->AddTerminalParameters("SensorHeight", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
//    cameraSettings->AddTerminalParameters("FOVWidth", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    cameraSettings->AddTerminalParameters("FOVHeight", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    cameraSettings->AddTerminalParameters("ImageWidth", MaceCore::ModuleParameterTerminalTypes::INT, false);
//    cameraSettings->AddTerminalParameters("ImageHeight", MaceCore::ModuleParameterTerminalTypes::INT, false);
//    cameraSettings->AddTerminalParameters("Frequency", MaceCore::ModuleParameterTerminalTypes::DOUBLE, false);
//    structure.AddNonTerminal("CameraParameters", cameraSettings, false);

    std::shared_ptr<MaceCore::ModuleParameterStructure> circularCameraSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    circularCameraSettings->AddTerminalParameters("CameraName", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    circularCameraSettings->AddTerminalParameters("ViewHalfAngle", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    circularCameraSettings->AddTerminalParameters("AlphaAttenuation", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    circularCameraSettings->AddTerminalParameters("BetaAttenuation", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    circularCameraSettings->AddTerminalParameters("CertainRangePercent", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    circularCameraSettings->AddTerminalParameters("P_D", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    circularCameraSettings->AddTerminalParameters("P_FA", MaceCore::ModuleParameterTerminalTypes::DOUBLE, true);
    structure.AddNonTerminal("CircularCameraParameters", circularCameraSettings, false);

    structure.AddTerminalParameters("TruthBTFile", MaceCore::ModuleParameterTerminalTypes::STRING, true);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleVehicleSensors::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
//    if(params->HasNonTerminal("CameraParameters"))
//    {
//        std::shared_ptr<MaceCore::ModuleParameterValue> protocolSettings = params->GetNonTerminalValue("CameraParameters");
//        cameraSensor->setCameraName(protocolSettings->GetTerminalValue<std::string>("CameraName"));
//        cameraSensor->setStabilization(true);
//        cameraSensor->setFocalLength(protocolSettings->GetTerminalValue<double>("FocalLength"));
//        cameraSensor->setSensorWidth(protocolSettings->GetTerminalValue<double>("SensorWidth"));
//        cameraSensor->setSensorHeight(protocolSettings->GetTerminalValue<double>("SensorHeight"));
//        cameraSensor->updateCameraProperties();
////        if(protocolSettings->HasTerminal("FOVWidth") && protocolSettings->HasTerminal("FOVHeight"))
////        {
////            cameraSensor->setFOV_Horizontal(protocolSettings->GetTerminalValue<double>("FOVWidth"));
////            cameraSensor->setFOV_Vertical(protocolSettings->GetTerminalValue<double>("FOVHeight"));
////        }else{
////            //update based on the sensor data
////            cameraSensor->updateCameraProperties();
////        }
////
//        if(protocolSettings->HasTerminal("ImageWidth"))
//            cameraSensor->setImageWidth(protocolSettings->GetTerminalValue<int>("ImageWidth"));
//        if(protocolSettings->HasTerminal("ImageHeight"))
//            cameraSensor->setImageHeight(protocolSettings->GetTerminalValue<int>("ImageHeight"));
//        if(protocolSettings->HasTerminal("Frequency"))
//            cameraSensor->setImageRate(protocolSettings->GetTerminalValue<double>("Frequency"));
//    }

    if(params->HasNonTerminal("CircularCameraParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> protocolSettings = params->GetNonTerminalValue("CircularCameraParameters");
        m_circularCameraSensor->setCameraName(protocolSettings->GetTerminalValue<std::string>("CameraName"));
        m_circularCameraSensor->setViewHalfAngle(protocolSettings->GetTerminalValue<double>("ViewHalfAngle"));
        m_circularCameraSensor->setAlphaAttenuation(protocolSettings->GetTerminalValue<double>("AlphaAttenuation"));
        m_circularCameraSensor->setBetaAttenuation(protocolSettings->GetTerminalValue<double>("BetaAttenuation"));
        m_circularCameraSensor->setCertainRangePercent(protocolSettings->GetTerminalValue<double>("CertainRangePercent"));
        m_circularCameraSensor->setProbDetection(protocolSettings->GetTerminalValue<double>("P_D"));
        m_circularCameraSensor->setProbFalseAlarm(protocolSettings->GetTerminalValue<double>("P_FA"));
    }
    if(params->HasTerminal("TruthBTFile"))
    {
        char* MACEPath = getenv("MACE_ROOT");
        std::string rootPath(MACEPath);
        const char kPathSeperator =
        #ifdef _WIN32
                '\\';
        #else
                '/';
        #endif
        //    std::string btFile = rootPath + kPathSeperator + "load_303030.bt";
        std::string btFile = params->GetTerminalValue<std::string>("TruthBTFile");
        m_truthBTFile = rootPath + kPathSeperator + btFile;
    }



    else {
        throw std::runtime_error("Unknown sensor parameters encountered");
    }
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
void ModuleVehicleSensors::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
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
void ModuleVehicleSensors::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    UNUSED(sender); UNUSED(target);

//    int senderID = sender.ModuleID;
    if(topicName == m_VehicleDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), sender);
        //example of how to get data and parse through the components that were updated
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
//            if(componentsUpdated.at(i) == DataStateTopic::StateAttitudeTopic::Name()) {
//                std::shared_ptr<DataStateTopic::StateAttitudeTopic> component = std::make_shared<DataStateTopic::StateAttitudeTopic>();
//                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
//            }
//            if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionExTopic::Name()) {
//                std::shared_ptr<DataStateTopic::StateGlobalPositionExTopic> component = std::make_shared<DataStateTopic::StateGlobalPositionExTopic>();
//                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
//                DataState::StateGlobalPositionEx newPosition = *component.get();
//                updateDataInSensorFootprint_Circular(newPosition);
//            }
        }
    }
}

//!
//! \brief computeVehicleFootprint Compute the vertices of the camera footprint and notify listeners of updated footprint
//! \param systemID Generating system ID
//! \param camera Camera properties
//! \param globalPosition Position of the vehicle/sensor
//! \param attitude Attitude of the vehicle/sensor
//!
void ModuleVehicleSensors::computeVehicleFootprint(const int &systemID, const DataVehicleSensors::SensorCamera &camera, const GeodeticPosition_3D &globalPosition, const Rotation_3D &attitude)
{
    UNUSED(camera); UNUSED(globalPosition); UNUSED(attitude); UNUSED(systemID);
//
//    DataState::StateGlobalPositionEx vehicleOrigin = globalPosition;

//    //This function also assumes that altitude is AGL and the surface below is relatively flat thus not requiring intersection calculations
//    //These calculations could be made however would require a model of the topography
//    //Vertice computation should always be done in a clockwise pattern starting with the upper right relative to vehicle heading
//    //The first check will be to see if the camera is stabilized
//    //If this is true, this allows us to easily compute a basic quadrilateral footprint

//
//    4 ------- 1
//     \       / -
//         X
//     /       \ -
//    3         2
//
//    mace::pose::GeodeticPosition_3D origin = this->getDataObject()->GetGlobalOrigin();
//    CommandItem::SpatialHome globalHome(origin);
//    DataState::StateGlobalPosition globalPos(globalHome.position->getX(),globalHome.position->getY(),globalHome.position->getZ());

//    std::vector<DataState::StateGlobalPosition> verticeVectorGlobal(4);
//    std::vector<DataState::StateLocalPosition> verticeVectorLocal(4);
//    if(camera.getStabilization())
//    {
//        double fovH = tan(camera.getFOV_Horizontal()/2) * 30;
//        double fovV = tan(camera.getFOV_Vertical()/2) * 30;
//        double verticeDistance = sqrt(fovH*fovH + fovV*fovV);
//        double bearing = atan2(fovH,fovV);

//        double currentHeading = globalPosition.heading;

//        DataState::StateGlobalPosition position1 = vehicleOrigin.NewPositionFromHeadingBearing(verticeDistance,bearing+currentHeading,false);
//        Eigen::Vector3f transVec1;
//        globalPos.translationTransformation3D(position1,transVec1);
//        verticeVectorGlobal[0] = position1;
//        DataState::StateLocalPosition position1L(transVec1(0),transVec1(1),transVec1(2));
//        verticeVectorLocal[0] = position1L;

//        DataState::StateGlobalPosition position2 = vehicleOrigin.NewPositionFromHeadingBearing(verticeDistance,currentHeading - bearing - 3.14159,false);
//        Eigen::Vector3f transVec2;
//        globalPos.translationTransformation3D(position2,transVec2);
//        verticeVectorGlobal[1] = position2;
//        DataState::StateLocalPosition position2L(transVec2(0),transVec2(1),transVec2(2));
//        verticeVectorLocal[1] = position2L;

//        DataState::StateGlobalPosition position3 = vehicleOrigin.NewPositionFromHeadingBearing(verticeDistance,bearing + currentHeading - 3.14159,false);
//        Eigen::Vector3f transVec3;
//        globalPos.translationTransformation3D(position3,transVec3);
//        verticeVectorGlobal[2] = position3;
//        DataState::StateLocalPosition position3L(transVec3(0),transVec3(1),transVec3(2));
//        verticeVectorLocal[2] = position3L;

//        DataState::StateGlobalPosition position4 = vehicleOrigin.NewPositionFromHeadingBearing(verticeDistance,currentHeading - bearing,false);
//        Eigen::Vector3f transVec4;
//        globalPos.translationTransformation3D(position4,transVec4);
//        verticeVectorGlobal[3] = position4;
//        DataState::StateLocalPosition position4L(transVec4(0),transVec4(1),transVec4(2));
//        verticeVectorLocal[3] = position4L;

//        std::shared_ptr<DataVehicleSensors::SensorVertices_Global> globalVert = std::make_shared<DataVehicleSensors::SensorVertices_Global>(camera.getCameraName());
//        globalVert->setSensorVertices(verticeVectorGlobal);

//        std::shared_ptr<DataVehicleSensors::SensorVertices_Local> localVert = std::make_shared<DataVehicleSensors::SensorVertices_Local>(camera.getCameraName());
//        localVert->setSensorVertices(verticeVectorLocal);

//        //Let us publish all of the information
//        MaceCore::TopicDatagram topicDatagram;

//        m_SensorFootprintDataTopic.SetComponent(globalVert, topicDatagram);
//        ModuleVehicleSensors::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
//            ptr->NewTopicDataValues(this, m_SensorFootprintDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
//        });

//        m_SensorFootprintDataTopic.SetComponent(localVert, topicDatagram);
//        ModuleVehicleSensors::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
//            ptr->NewTopicDataValues(this, m_SensorFootprintDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
//        });
//    }
}

//!
//! \brief loadTruthMap Load the truth map from a file
//! \param btFile Filename, relative to the root MACE path
//!
void ModuleVehicleSensors::loadBTFile(const std::string &btFilePath, const std::string &layerName) {

#ifdef __unix__
    // ** 1) Read in .bt file (If Windows, just skip and warn user that this is not supported in Windows)
//    char* MACEPath = getenv("MACE_ROOT");
//    std::string rootPath(MACEPath);
//    btFilePath = rootPath + kPathSeperator + "load_303030.bt";
    mace::maps::OctomapWrapper octomapWrapper;
    octomapWrapper.loadOctreeFromBT(btFilePath);

    // ** 2) Convert to Data2D Grid and insert into the m_LayeredMap_Truth container (with appropriate identifier layerName):
    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* compressedMap = octomapWrapper.get2DOccupancyMap();
    // Adjust for center position:
    compressedMap->updateOriginPosition(mace::pose::CartesianPosition_2D(-15,-15));

    // ** 3) Update truth map in this module with the truth map
    m_LayeredMap_Truth->updateMapLayer("truth_" + layerName, compressedMap);

    // ** 4) Update local map in mace_data size to whatever the truth layered map is:
    mace::maps::OccupiedResult fillValue = mace::maps::OccupiedResult::UNKNOWN;
    double xMin, yMin, xMax, yMax, xRes, yRes;
    xMin = compressedMap->getXMin();
    xMax = compressedMap->getXMax();
    yMin = compressedMap->getYMin();
    yMax = compressedMap->getYMax();
    xRes = compressedMap->getXResolution();
    yRes = compressedMap->getYResolution();
    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* tmpCompressedMap =
            new mace::maps::Data2DGrid<mace::maps::OccupiedResult>(&fillValue, xMin, xMax, yMin, yMax, xRes, yRes, compressedMap->getOriginPosition());

    std::string localLayerName = "local_" + layerName;
//    this->getDataObject()->updateLayeredMapLayer(localLayerName, tmpCompressedMap);


    // ** 5) Notify listeners of updated layered map layer:
    ModuleVehicleSensors::NotifyListeners([&](MaceCore::IModuleEventsSensors* ptr){
        ptr->Sensors_UpdatedMapLayer(localLayerName, tmpCompressedMap);
    }); //this one explicitly calls mace_core and its up to you to handle in core

#elif _WIN32
    MaceLog::Alert("Loading occupancy map from .bt file not supported in Windows...");
    UNUSED(btFilePath); UNUSED(layerName);
#endif
}

void ModuleVehicleSensors::loadRoadNetwork(const string &filePath, const string &layerName) {
    // TODO - Load road network from file and update the core with a local copy of the truth map
    //          - Store a truth copy in sensors module, and a local copy that vehicles update from sensor inputs in the core

    MaceLog::Alert("TODO: Implement road network graph load from file");
    UNUSED(filePath); UNUSED(layerName);
}

//!
//! \brief computeVehicleFootprint Compute the vertices of the camera footprint and notify listeners of updated footprint
//! \param systemID Generating system ID
//! \param camera Camera properties
//! \param globalPosition Position of the vehicle/sensor
//! \param attitude Attitude of the vehicle/sensor
//!
double ModuleVehicleSensors::computeVehicleFootprint_Circular(const DataVehicleSensors::SensorCircularCamera &camera, const CartesianPosition_3D &sensorOrigin) {
    // Use position and altitude to determine circular sensor footprint
    //  **  radius = height*tan(coneAngle)  **
    //      - Cone angle is the half angle

    double radius = sensorOrigin.getZPosition()*tan(camera.getViewHalfAngle());
    return radius;
}

//!
//! \brief updateDataInSensorFootprint_Circular Update the local map data from truth data in a circular footprint
//! \param sensorOriginGlobal Sensor origin for footprint calculations
//!
void ModuleVehicleSensors::updateDataInSensorFootprint_Circular(const mace::pose::GeodeticPosition_3D &sensorOriginGlobal) {
    // 1) Use sensor footprint to get data from truth map (loaded at startup?)
    //          - Use iterator
    // 2) Use truth data to update data in local data map
    //          - Use iterator? Update function?

    // // Example casting:
    //    mace::maps::BaseGridMap* map = m_LayeredMap_Local->getMapLayer(layerName);
    //    mace::maps::Data2DGrid<mace::maps::MapCell>* tmp = static_cast<mace::maps::Data2DGrid<mace::maps::MapCell>>(map);

    mace::maps::BaseGridMap* truthLayerBase = m_LayeredMap_Truth->getMapLayer("truth_OccupancyLayer");
    mace::maps::Data2DGrid<mace::maps::MapCell>* truthLayer = static_cast<mace::maps::Data2DGrid<mace::maps::MapCell>*>(truthLayerBase);
//    mace::maps::Data2DGrid<mace::maps::MapCell>* localLayer = this->getDataObject()->getLayeredMapLayer<mace::maps::Data2DGrid<mace::maps::MapCell>>("local_OccupancyLayer");
    const std::shared_ptr<mace::maps::LayeredMap> localLayeredMap = this->getDataObject()->getLayeredMap();
    mace::maps::BaseGridMap* map = localLayeredMap->getMapLayer("local_OccupancyLayer");
    mace::maps::Data2DGrid<mace::maps::MapCell>* localLayer = static_cast<mace::maps::Data2DGrid<mace::maps::MapCell>*>(map);

    // If we don't have any truth data, we can't update
    if(truthLayer->getNodeCount() > 0) {
        // Convert sensor global position to a Cartesian position:
        mace::pose::GeodeticPosition_3D globalOrigin = this->getDataObject()->GetGlobalOrigin();
        mace::pose::GeodeticPosition_3D originGlobal(sensorOriginGlobal.getLatitude(), sensorOriginGlobal.getLongitude(), sensorOriginGlobal.getAltitude());
        mace::pose::CartesianPosition_3D sensorOriginLocal;
        mace::pose::DynamicsAid::GlobalPositionToLocal(&globalOrigin, &originGlobal, &sensorOriginLocal);

        // Compute radius of sensor footprint based on sensor origin (altitude):
        double radius = computeVehicleFootprint_Circular(*m_circularCameraSensor, sensorOriginLocal);

        // Iterate over a circular footprint of radius = vehicle_footprint_radius:
        mace::pose::CartesianPosition_2D sensorOriginLocal_2D(sensorOriginLocal.getXPosition(), sensorOriginLocal.getYPosition());
        mace::maps::CircleMapIterator circleIt_truth(truthLayer, sensorOriginLocal_2D, radius);
        for(; !circleIt_truth.isPastEnd(); ++circleIt_truth)
        {
            // 1) Get value from truth map at iterator position from local map
            mace::maps::MapCell* truthVal = truthLayer->getCellByIndex(*circleIt_truth);
            double xPos, yPos;
            truthLayer->getPositionFromIndex(*circleIt_truth, xPos, yPos);

            // 2) Calculate distance from sensor origin for attenuated disk:
            CartesianPosition_2D currentPosition(xPos, yPos);
            double distanceToSensorOrigin = sensorOriginLocal.distanceBetween2D(&currentPosition);
            double sigma = m_circularCameraSensor->attenuatedDiskConfidence(distanceToSensorOrigin, radius);

            // 3) Get occupied value to update log odds and update our log odds probability:
            mace::maps::MapCell* localVal = localLayer->getCellByPos(xPos, yPos);
            mace::maps::OccupiedResult occupied = truthVal->getCellValue();
            double priorLogOdds = localVal->getLogOdds();
            localVal->updateLogOddsProbability(occupied, m_circularCameraSensor->getProbDetection(), m_circularCameraSensor->getProbFalseAlarm(), sigma);
            double newLogOdds = localVal->getLogOdds();

            // 4) Update local map with truth value and current time:
            Data::EnvironmentTime now;
            Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, now);
            if((newLogOdds > 0 && occupied == mace::maps::OccupiedResult::OCCUPIED) ||
               (newLogOdds > 0 && occupied == mace::maps::OccupiedResult::NOT_OCCUPIED) ||
               (newLogOdds < 0 && priorLogOdds > 0 && occupied == mace::maps::OccupiedResult::OCCUPIED)) {
                //      - If new log-odds value is positive and measured value was OCCUPIED: set local value to OCCUPIED
                //      - If new log-odds value is positive and the measured value was NOT_OCCUPIED: leave local value as OCCUPIED
                //      - If new log-odds value is negative, the previous log-odds was positive, and the measured value was OCCUPIED: leave local value as OCCUPIED
                localVal->setCellValue(mace::maps::OccupiedResult::OCCUPIED, now);
            }
            else if((newLogOdds < 0 && priorLogOdds < 0 && occupied == mace::maps::OccupiedResult::OCCUPIED) ||
                    (newLogOdds < 0 && occupied == mace::maps::OccupiedResult::NOT_OCCUPIED)) {

                //      - If new log-odds value is negative, the previous log-odds was negative, and the measured value was OCCUPIED: set local value to NOT_OCCUPIED
                //      - If new log-odds value is negative, and the measured value is NOT_OCCUPIED: set the local value to NOT_OCCUPIED
                localVal->setCellValue(mace::maps::OccupiedResult::NOT_OCCUPIED, now);
            }
            else {
                // Don't update local value
            }
        }


        // Update local occupancy layer in core:
        this->getDataObject()->updateLayeredMapLayer("local_OccupancyLayer", localLayer);
    }
    else {
        std::cout << "Truth map has no data. Nothing to do." << std::endl;
    }

}



//!
//! \brief NewlyAvailableVehicle Subscriber to a newly available vehicle topic
//! \param vehicleID Vehilce ID of the newly available vehicle
//!
void ModuleVehicleSensors::NewlyAvailableVehicle(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(vehicleID); UNUSED(sender);
}

//!
//! \brief NewlyAvailableGlobalOrigin Subscriber to a new global origin
//!
void ModuleVehicleSensors::NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &globalOrigin)
{
    UNUSED(globalOrigin);

    std::cout << "Sensors: New available global origin" << std::endl;
}

//!
//! \brief OnModulesStarted Method that fires when all modules have started
//!
void ModuleVehicleSensors::OnModulesStarted()
{
    std::cout << "All of the modules have been started." << std::endl;

    // TESTING:
//    double minX = -50.0; double maxX = 50.0;
//    double minY = -50.0; double maxY = 50.0;
//    double x_res = 10.0; double y_res = 10.0;
//    mace::maps::MapCell truthFill(mace::maps::OccupiedResult::NOT_OCCUPIED, 1.0, false);
//    mace::maps::MapCell localFill(mace::maps::OccupiedResult::UNKNOWN, 0.0, false);
//    // TODO: Load this from config file?
////    mace::maps::OccupiedResult truthFill = mace::maps::OccupiedResult::NOT_OCCUPIED;
////    mace::maps::OccupiedResult localFill = mace::maps::OccupiedResult::UNKNOWN;
//    m_compressedMapTruth = new mace::maps::Data2DGrid<mace::maps::MapCell>(&truthFill, minX, maxX, minY, maxY, x_res, y_res);
//    mace::maps::Data2DGrid<mace::maps::MapCell>* localMap = new mace::maps::Data2DGrid<mace::maps::MapCell>(&localFill, minX, maxX, minY, maxY, x_res, y_res);
//    // END TESTING (todo: initialize maps when not testing)

    m_LayeredMap_Truth = std::make_shared<mace::maps::LayeredMap>();
//    m_LayeredMap_Truth->updateMapLayer("truth_OccupancyLayer", m_compressedMapTruth);
//    this->getDataObject()->updateLayeredMapLayer("local_OccupancyLayer", localMap);

    // Load truth map:
    loadBTFile(m_truthBTFile, "OccupancyLayer");
}
