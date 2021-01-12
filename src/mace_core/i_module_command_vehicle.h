#ifndef I_MODULE_COMMAND_VEHICLE_H
#define I_MODULE_COMMAND_VEHICLE_H

#include "abstract_module_base_vehicle_listener.h"
#include "metadata_vehicle.h"

#include "i_module_topic_events.h"
#include "i_module_events_vehicle.h"

#include "data_generic_command_item/command_item_components.h"
#include "base/pose/pose.h"

#include "i_module_command_ai_support.h"

namespace MaceCore
{

enum class VehicleCommands
{
    BASE_MODULE_LISTENER_ENUMS,
    BASE_MODULE_VEHICLE_LISTENER_ENUMS,
    AI_SUPPORT_GENERAL_COMMAND_ENUMS,
    AI_SUPPORT_VEHICLE_COMMAND_ENUMS,
    REQUEST_DUMMY_FUNCTION,
    UPDATE_MISSION_KEY,
    UPDATED_DYNAMIC_MISSION_QUEUE,
    TRANSMIT_VISION_POSE_ESTIMATE
};


class MaceCore;

class MACE_CORESHARED_EXPORT IModuleCommandVehicle : public AbstractModule_VehicleListener<MetadataVehicle, IModuleEventsVehicle, VehicleCommands>,
        public IModuleCommand_VehicleAISupport
{
friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandVehicle():
        AbstractModule_VehicleListener(),
        IModuleCommand_VehicleAISupport()
    {
        IModuleCommand_VehicleAISupport::SetUp<MetadataVehicle, IModuleEventsVehicle, VehicleCommands>(this);

        //These are from MACE Core to modules
        this->template AddCommandLogic<int>(VehicleCommands::REQUEST_DUMMY_FUNCTION, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            RequestDummyFunction(vehicleID);
        });

        this->template AddCommandLogic<MissionItem::MissionKeyChange>(VehicleCommands::UPDATE_MISSION_KEY, [this](const MissionItem::MissionKeyChange &key, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            UpdateMissionKey(key);
        });

        this->template AddCommandLogic<command_target::DynamicMissionQueue>(VehicleCommands::UPDATED_DYNAMIC_MISSION_QUEUE, [this](const command_target::DynamicMissionQueue &queue, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            UpdateDynamicMissionQueue(queue);
        });

        this->template AddCommandLogic<mace::pose::Pose>(VehicleCommands::TRANSMIT_VISION_POSE_ESTIMATE, [this](const mace::pose::Pose &pose, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            TransmitVisionPoseEstimate(pose);
        });

    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }


public:
    //!
    //! \brief RequestDummyFunction Test function
    //! \param vehicleID Vehicle ID
    //!
    virtual void RequestDummyFunction(const int &vehicleID) = 0;

    //!
    //! \brief UpdateMissionKey Update mission key command
    //! \param key Mission key to update
    //!
    virtual void UpdateMissionKey(const MissionItem::MissionKeyChange &key) = 0;

    //!
    //! \brief UpdateDynamicMissionQueue Update dynamic mission queue command
    //! \param queue Mission queue
    //!
    virtual void UpdateDynamicMissionQueue(const command_target::DynamicMissionQueue &queue) = 0;

    //!
    //! \brief TransmitVisionPoseEstimate
    //! \param pose
    //!
    virtual void TransmitVisionPoseEstimate(const mace::pose::Pose &pose) = 0;

};


} //END MaceCore Namespace

#endif // I_MODULE_COMMAND_VEHICLE_H
