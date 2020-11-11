#include "module_vehicle_mavlink.h"
template <typename ...VehicleTopicAdditionalComponents>
void ModuleVehicleMAVLINK<VehicleTopicAdditionalComponents...>::handleFirstConnectionSetup()
{
    if(setEKFOrigin == true)
    {
        command_item::Action_SetGlobalOrigin command_EKFOrigin;
        command_EKFOrigin.setOriginatingSystem(255);
        command_EKFOrigin.setTargetSystem(m_SystemData->getMAVLINKID());
        command_EKFOrigin.setGlobalOrigin(&desiredEKFPosition);
        handleGlobalOriginController(command_EKFOrigin);
    }
    if(setEKFasHOME == true)
    {
        command_item::SpatialHome command_SystemHome;
        command_SystemHome.setOriginatingSystem(255);
        command_SystemHome.setTargetSystem(m_SystemData->getMAVLINKID());
        command_SystemHome.setPosition(&desiredEKFPosition);
        handleHomePositionController(command_SystemHome);
    }
    else
    {
        //Since this is the first time, we also should request to see if the vehicle has already established a home position
        command_item::SpatialHome homeRequest(255,m_SystemData->getMAVLINKID());
        homeRequest.setActionType(command_item::AbstractCommandItem::GET_COMMAND);
        handleHomePositionController(homeRequest);
    }
    //request the current mission on the vehicle
    this->prepareMissionController();
    MAVLINKUXVControllers::ControllerMission* missionController = static_cast<MAVLINKUXVControllers::ControllerMission*>(m_ControllersCollection.At("missionController"));
    if(missionController)
    {
        missionController->AddLambda_Finished(this, [missionController](const bool completed, const uint8_t code){
            UNUSED(code);
            if (completed)
                printf("Mission Download Completed\n");
            else
                printf("Mission Download Failed\n");
            //Ken Fix: We should handle stuff related to the completion and the code
            missionController->Shutdown();
        });
        missionController->GetMissions(m_SystemData->getMAVLINKID());
    }
    //request the current parameters on the vehicle (for now we do just one)
    this->prepareParameterController();
    MAVLINKUXVControllers::Controller_ParameterRequest* parameterController = static_cast<MAVLINKUXVControllers::Controller_ParameterRequest*>(m_ControllersCollection.At("parameterController"));
    if(parameterController)
    {
        parameterController->AddLambda_Finished(this, [this, parameterController](const bool completed, const MAVLINKUXVControllers::ParameterRequestResult &data){
            if (completed)
                printf("Parameter Download Completed\n");
            else
                printf("Parameter Download Failed\n");

            DataGenericItem::DataGenericItem_ParamValue parameterValue;
            parameterValue.setParamValues(data.parameterIndex, data.parameterName, data.parameterValue);
            //notify the core of the change
            ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
                ptr->GVEvents_NewParameterValue(this, parameterValue);
            });

            //Ken Fix: We should handle stuff related to the completion and the code
            parameterController->Shutdown();
        });
        MavlinkEntityKey target = this->GetAttachedMavlinkEntity();
        MavlinkEntityKey sender = 255;
        MAVLINKUXVControllers::ParameterRequestResult parameterRequest;
        parameterRequest.vehicleID = this->GetAttachedMavlinkEntity();
        parameterRequest.parameterName = "TKOFF_ALT";
        parameterController->Send(parameterRequest, sender, target);
    }
}
template <typename ...VehicleTopicAdditionalComponents>
void ModuleVehicleMAVLINK<VehicleTopicAdditionalComponents...>::handleHomePositionController(const command_item::SpatialHome &commandObj)
{
    if(m_SystemData != nullptr) // this means that this module currently has a vehicle
    {
        command_item::AbstractCommandItem::getORset currentAction = commandObj.getActionType();
        if(m_ControllersCollection.Exist("homePositionController"))
        {
            std::cout << "Shutting down the previous home controller that was already interacting with information we were working with." << std::endl;
            if(currentAction == command_item::AbstractCommandItem::GET_COMMAND)
            {
                auto homePositionOld = static_cast<MAVLINKUXVControllers::Command_HomePositionGet*>(m_ControllersCollection.At("homePositionController"));
                if(homePositionOld != nullptr)
                {
                    homePositionOld->Shutdown();
                    // Need to wait for the old controller to be shutdown.
                    std::unique_lock<std::mutex> controllerLock(m_mutex_HomePositionController);
                    while (!m_oldHomePositionControllerShutdown)
                        m_condition_HomePositionController.wait(controllerLock);
                    m_oldHomePositionControllerShutdown = false;
                    controllerLock.unlock();
                }
            }
            else if(currentAction == command_item::AbstractCommandItem::SET_COMMAND)
            {
                auto homePositionOld = static_cast<MAVLINKUXVControllers::Command_HomePositionSet*>(m_ControllersCollection.At("homePositionController"));
                if(homePositionOld != nullptr)
                {
                    homePositionOld->Shutdown();
                    // Need to wait for the old controller to be shutdown.
                    std::unique_lock<std::mutex> controllerLock(m_mutex_HomePositionController);
                    while (!m_oldHomePositionControllerShutdown)
                        m_condition_HomePositionController.wait(controllerLock);
                    m_oldHomePositionControllerShutdown = false;
                    controllerLock.unlock();
                }
            }
        }
        if(currentAction == command_item::AbstractCommandItem::GET_COMMAND)
        {
            auto homePositionController = new MAVLINKUXVControllers::Command_HomePositionGet(m_SystemData, m_TransmissionQueue, m_LinkChan);
            //create "stateless" global origin controller that exists within the module itself
            homePositionController->AddLambda_Finished(this, [this, homePositionController](const bool completed, const uint8_t finishCode){
                UNUSED(finishCode);
                homePositionController->Shutdown();
                if(!completed)
                    return;
            });
            homePositionController->setLambda_Shutdown([this]()
            {
                auto ptr = m_ControllersCollection.Remove("homePositionController");
                delete ptr;
            });
            MavlinkEntityKey target = this->GetAttachedMavlinkEntity();
            MavlinkEntityKey sender = 255;
            command_item::SpatialHome objCopy(commandObj);
            objCopy.setTargetSystem(static_cast<uint8_t>(target));
            objCopy.setOriginatingSystem(static_cast<uint8_t>(sender));
            homePositionController->Send(objCopy, sender, target);
            m_ControllersCollection.Insert("homePositionController", homePositionController);
        }
        else if(currentAction == command_item::AbstractCommandItem::SET_COMMAND)
        {
            auto homePositionController = new MAVLINKUXVControllers::Command_HomePositionSet(m_SystemData, m_TransmissionQueue, m_LinkChan);
            //create "stateless" global origin controller that exists within the module itself
            homePositionController->AddLambda_Finished(this, [this, homePositionController](const bool completed, const uint8_t finishCode){
                UNUSED(finishCode);
                homePositionController->Shutdown();
                if(!completed)
                    return;
            });
            homePositionController->setLambda_Shutdown([this]()
            {
                auto ptr = m_ControllersCollection.Remove("homePositionController");
                delete ptr;
            });
            MavlinkEntityKey target = this->GetAttachedMavlinkEntity();
            MavlinkEntityKey sender = 255;
            command_item::SpatialHome objCopy(commandObj);
            objCopy.setTargetSystem(static_cast<uint8_t>(target));
            objCopy.setOriginatingSystem(static_cast<uint8_t>(sender));
            homePositionController->Send(objCopy, sender, target);
            m_ControllersCollection.Insert("homePositionController", homePositionController);
        }
    }
}
template <typename ...VehicleTopicAdditionalComponents>
void ModuleVehicleMAVLINK<VehicleTopicAdditionalComponents...>::handleGlobalOriginController(const command_item::Action_SetGlobalOrigin &originObj)
{
    if((m_SystemData != nullptr) && (originObj.getGlobalOrigin()->areTranslationalComponentsValid())) // this means that this module currently has a vehicle
    {
        if(m_ControllersCollection.Exist("swarmOriginController"))
        {
            auto globalOriginOld = static_cast<MAVLINKUXVControllers::Controller_SetGPSGlobalOrigin*>(m_ControllersCollection.At("Controllers_GlobalOrigin"));
            if(globalOriginOld != nullptr)
            {
                std::cout << "Shutting down the previous global origin controller which was still active prior to receiving this update." << std::endl;
                globalOriginOld->Shutdown();
                // Need to wait for the old controller to be shutdown.
                std::unique_lock<std::mutex> controllerLock(m_mutex_GlobalOriginController);
                while (!m_oldGlobalOriginControllerShutdown)
                    m_condition_GlobalOriginController.wait(controllerLock);
                m_oldGlobalOriginControllerShutdown = false;
                controllerLock.unlock();
            }
        }
        //create "stateless" global origin controller that exists within the module itself
        auto globalOriginController = new MAVLINKUXVControllers::Controller_SetGPSGlobalOrigin(m_SystemData, m_TransmissionQueue, m_LinkChan);
        globalOriginController->AddLambda_Finished(this, [this, globalOriginController](const bool completed, const mace::pose::GeodeticPosition_3D receivedOrigin){
            if(completed)
            {
                //The received origin from the autopilot is the one it will reference
                if(this->m_SystemData != nullptr) //because this is a lambda during a callback in future this needs to be checked
                    this->m_SystemData->environment->vehicleGlobalOrigin.set(receivedOrigin);
            }
            else
            {
                std::thread thread([this](){
                    command_item::ActionMessageRequest messageRequest(255,m_SystemData->getMAVLINKID());
                    messageRequest.setMessageID(MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN);
                    MAVLINKUXVControllers::CommandMSGRequest* newMSGRequest = new MAVLINKUXVControllers::CommandMSGRequest(m_SystemData, m_TransmissionQueue, m_LinkChan);
                    newMSGRequest->AddLambda_Finished(this,[this,newMSGRequest](const bool completed, const uint8_t finishCode)
                    {
                        UNUSED(completed); UNUSED(finishCode);
                        newMSGRequest->Shutdown();
                    });
                    newMSGRequest->setLambda_Shutdown([this](){
                        auto ptr = m_ControllersCollection.Remove("messageRequest");
                        delete ptr;
                    });
                    MavlinkEntityKey target = m_SystemData->getMAVLINKID();
                    MavlinkEntityKey sender = 255;
                    m_ControllersCollection.Insert("messageRequest", newMSGRequest);
                    newMSGRequest->Send(messageRequest,sender,target);
                });
                thread.detach();
            }
            globalOriginController->Shutdown();
        });
        globalOriginController->setLambda_Shutdown([this]()
        {
            auto ptr = m_ControllersCollection.Remove("swarmOriginController");
            delete ptr;
        });
        MavlinkEntityKey target = this->GetAttachedMavlinkEntity();
        MavlinkEntityKey sender = 255;
        command_item::Action_SetGlobalOrigin originCopy(originObj);
        originCopy.setTargetSystem(target);
        originCopy.setOriginatingSystem(sender);
        m_ControllersCollection.Insert("swarmOriginController", globalOriginController);
        globalOriginController->Send(originCopy, sender, target);
    }
}
template <typename ...VehicleTopicAdditionalComponents>
void ModuleVehicleMAVLINK<VehicleTopicAdditionalComponents...>::prepareMissionController()
{
    //If there is an old mission controller still running, allow us to shut it down
    if(m_ControllersCollection.Exist("missionController"))
    {
        auto missionControllerOld = static_cast<MAVLINKUXVControllers::ControllerMission*>(m_ControllersCollection.At("missionController"));
        if(missionControllerOld != nullptr)
        {
            std::cout << "Shutting down previous mission controller, which was still active" << std::endl;
            missionControllerOld->Shutdown();
            // Need to wait for the old controller to be shutdown.
            std::unique_lock<std::mutex> controllerLock(m_mutex_MissionController);
            while (!m_oldMissionControllerShutdown)
                m_condition_MissionController.wait(controllerLock);
            m_oldMissionControllerShutdown = false;
            controllerLock.unlock();
        }
    }
    //create "stateless" parameter controller that exists within the module itself
    auto missionController = new MAVLINKUXVControllers::ControllerMission(m_SystemData, m_TransmissionQueue, m_LinkChan);
    missionController->setLambda_DataReceived([this](const MavlinkEntityKey key, const MAVLINKUXVControllers::MissionDownloadResult &data){
        UNUSED(key);
        //////////////////////////////
        ///Update about Home position
        //////////////////////////////
        command_item::SpatialHome home = std::get<0>(data);
        mace::pose::GeodeticPosition_3D* homePosition = home.getPosition()->positionAs<mace::pose::GeodeticPosition_3D>();
        m_SystemData->environment->vehicleGlobalHome.set(*homePosition);
        this->cbi_VehicleHome(home.getOriginatingSystem(),home);
        //////////////////////////////
        ///Update about mission list
        //////////////////////////////
        MissionItem::MissionList missionList = std::get<1>(data);
        missionList.setVehicleID(this->GetAttachedMavlinkEntity());
        m_SystemData->mission->currentAutoMission.set(missionList);
        this->cbi_VehicleMission(missionList.getVehicleID(),missionList);
    });
    missionController->setLambda_Shutdown([this, missionController]() mutable
    {
        auto ptr = static_cast<MAVLINKUXVControllers::ControllerMission*>(m_ControllersCollection.At("missionController"));
        if (ptr == missionController && ptr != nullptr)
        {
            m_ControllersCollection.Remove("missionController");
            delete ptr;
            missionController = nullptr;
        }
        std::lock_guard<std::mutex> guard(m_mutex_MissionController);
        m_oldMissionControllerShutdown = true;
        m_condition_MissionController.notify_one();
    });
    m_ControllersCollection.Insert("missionController",missionController);
}
template <typename ...VehicleTopicAdditionalComponents>
void ModuleVehicleMAVLINK<VehicleTopicAdditionalComponents...>::prepareParameterController()
{
    //If there is an old parameter controller still running, allow us to shut it down
    if(m_ControllersCollection.Exist("parameterController"))
    {
        auto parameterControllerOld = static_cast<MAVLINKUXVControllers::Controller_ParameterRequest*>(m_ControllersCollection.At("parameterController"));
        if(parameterControllerOld != nullptr)
        {
            std::cout << "Shutting down previous parameter controller, which was still active" << std::endl;
            parameterControllerOld->Shutdown();
            // Need to wait for the old controller to be shutdown.
            std::unique_lock<std::mutex> controllerLock(m_mutex_ParameterController);
            while (!m_oldParameterControllerShutdown)
                m_condition_ParameterController.wait(controllerLock);
            m_oldParameterControllerShutdown = false;
            controllerLock.unlock();
        }
    }
    //create "stateless" parameter controller that exists within the module itself
    auto parameterController = new MAVLINKUXVControllers::Controller_ParameterRequest(m_SystemData, m_TransmissionQueue, m_LinkChan);
    parameterController->setLambda_Shutdown([this, parameterController]() mutable
    {
        auto ptr = static_cast<MAVLINKUXVControllers::Controller_ParameterRequest*>(m_ControllersCollection.At("parameterController"));
        if (ptr != nullptr)
        {
            auto ptr = m_ControllersCollection.Remove("parameterController");
            delete ptr;
        }
        std::lock_guard<std::mutex> guard(m_mutex_ParameterController);
        m_oldParameterControllerShutdown = true;
        m_condition_ParameterController.notify_one();
    });
    m_ControllersCollection.Insert("parameterController", parameterController);
}
