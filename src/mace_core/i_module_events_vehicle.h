#ifndef I_MODULE_VEHICLE_EVENTS_H
#define I_MODULE_VEHICLE_EVENTS_H

#include "i_module_events_general_vehicle.h"

#include "abstract_module_base.h"

namespace MaceCore
{

class IModuleEventsVehicle : virtual public IModuleEventsGeneralVehicle, virtual public IModuleEventsGeneral
{
public:
    //A vehicle module can indicate something has happened

    //!
    //! \brief EventVehcile_NewOnboardVehicleMission This virtual function event is used to indicate that a vehicle has a new
    //! onboard available mission. The type will be described in the key of the MissionItem but should be of type AUTO.
    //! \param sender
    //! \param missionList
    //!
    virtual void EventVehicle_NewOnboardVehicleMission(const ModuleBase *sender, const MissionItem::MissionList &missionList) = 0;

    //!
    //! \brief EventVehicle_MissionACK
    //! \param sender
    //! \param ack
    //!
    virtual void EventVehicle_MissionACK(const void *sender, const MissionItem::MissionACK &ack) = 0;

    //!
    //! \brief EventVehicle_REJECTProposedMission Event to trigger a rejected mission action
    //! \param sender Sender module
    //! \param key Rejected mission key
    //!
    virtual void EventVehicle_REJECTProposedMission(const void *sender, const MissionItem::MissionKey &key) = 0;

    //!
    //! \brief EventAI_ExecuteTestProcedural
    //! \param sender
    //! \param obj
    //!
//    virtual void EventVehicle_ExecuteAITestProcedural(const ModuleBase* sender, const command_item::Action_ProceduralCommand &obj) = 0;
};

} //End MaceCore Namespace

#endif // I_MODULE_VEHICLE_EVENTS_H
