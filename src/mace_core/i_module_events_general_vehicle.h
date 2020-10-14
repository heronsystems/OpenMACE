#ifndef I_MODULE_EVENTS_GENERAL_VEHICLE_H
#define I_MODULE_EVENTS_GENERAL_VEHICLE_H

#include "topic.h"

#include "data/mission_execution_state.h"
#include "data_generic_command_item/command_item_components.h"
#include "data_generic_item/data_generic_item_components.h"

#include "i_module_events_general.h"

//A vehicle module can indicate something has happened

namespace MaceCore
{

class IModuleEventsGeneralVehicle
{
public:
    virtual ~IModuleEventsGeneralVehicle() = default;

public:

    //!
    //! \brief GVEvents_NewHomePosition This function is emitted to alert the core that a module connected to a vehicle
    //! has received and set a new home position for the system. This should typically be in response to a Command_SetHomePosition
    //! request.
    //! \param sender
    //! \param vehicleHome
    //!
    virtual void GVEvents_NewHomePosition(const ModuleBase *sender, const command_item::SpatialHome &vehicleHome) = 0;

    //!
    //! \brief GVEvents_NewParameterValue This function is emitted to alert other modules of the current limitations of the vehicle
    //! \param sender
    //! \param vehicleParams
    //!
    virtual void GVEvents_NewParameterValue(const ModuleBase *sender, const DataGenericItem::DataGenericItem_ParamValue &vehicleParam) = 0;


    //!
    //! \brief GVEvents_MissionExecutionStateUpdated
    //! \param sender
    //! \param missionKey
    //! \param missionExeState
    //!
    virtual void GVEvents_MissionExeStateUpdated(const void *sender, const MissionItem::MissionKey &missionKey, const Data::MissionExecutionState &missionExeState) = 0;

    //!
    //! \brief ConfirmedOnboardVehicleMission
    //! \param sender
    //! \param missionKey
    //!
    virtual void ConfirmedOnboardVehicleMission(const void *sender, const MissionItem::MissionKey &missionKey) = 0;

    //!
    //! \brief UpdateCurrentVehicleMission
    //! \param sender
    //! \param systemID
    //! \param missionID
    //!
    virtual void NewCurrentVehicleMission(const void *sender, const MissionItem::MissionKey &missionKey) = 0;

    //!
    //! \brief GVEvents_MissionItemAchieved
    //! \param sender
    //! \param achieved
    //!
    virtual void GVEvents_MissionItemAchieved(const void *sender, const MissionItem::MissionItemAchieved &achieved) = 0;

    //!
    //! \brief GVEvents_MissionItemCurrent
    //! \param sender
    //! \param current
    //!
    virtual void GVEvents_MissionItemCurrent(const void *sender, const MissionItem::MissionItemCurrent &current) = 0;

    //!
    //! \brief GVEvents_NewSystemTime Emitted to alert the core that a module connected to a vehicle has an updated system time
    //! \param sender Sender module
    //! \param systemTime New system time
    //!
    virtual void GVEvents_NewSystemTime(const ModuleBase *sender, const DataGenericItem::DataGenericItem_SystemTime &systemTime) = 0;

};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_GENERAL_VEHICLE_H
