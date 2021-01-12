#ifndef MAVLINK_VEHICLE_OBJECT_H
#define MAVLINK_VEHICLE_OBJECT_H

#include "commsMAVLINK/comms_mavlink.h"

#include "controllers/generic_controller.h"

#include "../controllers/commands/command_arm.h"
#include "../controllers/commands/command_land.h"
#include "../controllers/commands/command_msg_interval.h"
#include "../controllers/commands/command_takeoff.h"
#include "../controllers/commands/command_rtl.h"
#include "../controllers/controller_system_mode.h"
#include "../controllers/controller_collection.h"

#include "state_data_mavlink.h"
#include "../environment_object/environment_data_mavlink.h"
#include "status_data_mavlink.h"
#include "mission_data_mavlink.h"

#include "base/vehicle/vehicle_path_linear.h"

#include "module_vehicle_MAVLINK/mavlink_entity_key.h"

template <typename T, typename TT>
T* Helper_CreateAndSetUp(TT* obj, TransmitQueue *queue, uint8_t chan)
{
    T* newController = new T(obj, queue, chan);
    //newController->setLambda_DataReceived([obj](const MaceCore::ModuleCharacteristic &sender, const std::shared_ptr<AbstractCommandItem> &command){obj->ReceivedCommand(sender, command);});
    //newController->setLambda_Finished(FinishedMAVLINKMessage);
    return newController;
}

class CallbackInterface_MAVLINKVehicleObject
{
public:
    virtual ~CallbackInterface_MAVLINKVehicleObject() = default;

public:
//    virtuul void cbi_GPSData(const int &systemID, std::shared_ptr<> data) = 0;
    virtual void cbi_VehicleStateData(const int &systemID, std::shared_ptr<Data::ITopicComponentDataObject> data) = 0;
    virtual void cbi_VehicleMissionData(const int &systemID, std::shared_ptr<Data::ITopicComponentDataObject> data) const = 0;

    virtual void cbi_VehicleSystemTime(const int &systemID, std::shared_ptr<DataGenericItem::DataGenericItem_SystemTime> systemTime) = 0;
    virtual void cbi_VehicleHome(const int &systemID, const command_item::SpatialHome &home) = 0;
    virtual void cbi_VehicleMission(const int &systemID, const MissionItem::MissionList &missionList) = 0;
    virtual void cbi_VehicleMissionItemCurrent(const MissionItem::MissionItemCurrent &current) const = 0;
    virtual void cbi_VehicleTrajectory(const int &systemID, const VehiclePath_Linear &trjectory) const = 0;
};

class MavlinkVehicleObject : public Controllers::IMessageNotifier<mavlink_message_t, int>
{
public:
    MavlinkVehicleObject(CommsMAVLINK *commsObj, const MaceCore::ModuleCharacteristic &module, const int &mavlinkID);

    virtual ~MavlinkVehicleObject();

    int getMAVLINKID() const;

    MaceCore::ModuleCharacteristic getModule() const;

    CommsMAVLINK* getCommsObject() const;

    void connectCallback(CallbackInterface_MAVLINKVehicleObject *cb)
    {
        m_CB = cb;
    }

    //!
    //! \brief TransmitMessage
    //! \param msg Message to transmit
    //! \param target Target to transmit to. Broadcast if not set.
    //!
    virtual void TransmitMessage(const mavlink_message_t &msg, const OptionalParameter<int> &target) const
    {
        UNUSED(target);
        commsLink->TransmitMAVLINKMessage(msg);
    }

    virtual int GetKeyFromSecondaryID(int ID) const
    {
        if(mavlinkID != ID)
        {
            throw std::runtime_error("Asked a local vehicle object for the ModuleCharacterstic for a vehicle that isn't it");
        }

        return mavlinkID;
    }


    virtual int GetHostKey() const
    {
        return mavlinkID;
    }


    virtual bool parseMessage(const mavlink_message_t *msg);


    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey>* ControllersCollection()
    {
        return &m_ControllersCollection;
    }

    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey>* GlobalControllersCollection()
    {
        return m_GlobalControllerCollection;
    }

    void linkToStatelessControllers(Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey>* controllerCollection)
    {
        m_GlobalControllerCollection = controllerCollection;
    }

    TransmitQueue *GetControllerQueue()
    {
        return controllerQueue;
    }

    const CallbackInterface_MAVLINKVehicleObject* getCallbackInterface() const
    {
        if(m_CB != nullptr)
            return m_CB;
    }

    bool handleMAVLINKMessage(const mavlink_message_t &msg);



private:
    static const uint16_t IGNORE_POS_TYPE_MASK = POSITION_TARGET_TYPEMASK_X_IGNORE|POSITION_TARGET_TYPEMASK_Y_IGNORE|POSITION_TARGET_TYPEMASK_Z_IGNORE;
    static const uint16_t IGNORE_VEL_TYPE_MASK = POSITION_TARGET_TYPEMASK_VX_IGNORE|POSITION_TARGET_TYPEMASK_VY_IGNORE|POSITION_TARGET_TYPEMASK_VZ_IGNORE;

public:
    Data::DataGetSetNotifier<Data::MACEHSMState> _currentHSMState;

public:
    StateData_MAVLINK *state;
    EnvironmentData_MAVLINK *environment;
    StatusData_MAVLINK *status;
    MissionData_MAVLINK *mission;

protected:
    int mavlinkID;
    MaceCore::ModuleCharacteristic m_module;

    PointerCollection<> m_Controllers;

    CommsMAVLINK *commsLink;

    CallbackInterface_MAVLINKVehicleObject* m_CB;

    TransmitQueue *controllerQueue;

    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> m_ControllersCollection;
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey>* m_GlobalControllerCollection;

    Data::EnvironmentTime prevAttitude;
    Data::EnvironmentTime prevPosition;

};

#endif // MAVLINK_VEHICLE_OBJECT_H
