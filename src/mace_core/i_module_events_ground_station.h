#ifndef I_GROUND_STATION_EVENTS_H
#define I_GROUND_STATION_EVENTS_H

#include "i_module_events_ai_support.h"
#include "i_module_events_general.h"
#include "i_module_events_boundary_generator.h"
#include "i_module_events_path_planning.h"

namespace MaceCore
{

class IModuleEventsGroundStation : virtual public IModuleEventsPathPlanning, virtual public IModuleEventsGeneral, virtual public IModuleEventsBoundaryGenerator
{
public:
    //!
    //! \brief RequestDummyFunction Test function
    //! \param sender Sender module
    //! \param vehicleID Vehicle ID
    //!
    virtual void RequestDummyFunction(const void* sender, const int &vehicleID) = 0;

    //!
    //! \brief Event_UploadMission method calls the appropriate handling operations to migrate the proposed
    //! mission list to the appropriate vehicle module for handling.
    //! \param sender
    //! \param missionList
    //!
    virtual void GSEvent_UploadMission(const void* sender, const MissionItem::MissionList &missionList) = 0;
};

class IModuleEventsMLStation: virtual public IModuleEventsGroundStation, virtual public IModuleEvents_AISupport
{
public:

    //!
    //! \brief Event_StartRound Event to trigger and new testing round
    //! \param sender Sender module
    //! \param command test setup information
    //!
    //virtual void Event_StartRound(const ModuleBase* sender, const DataGenericItem::DataGenericItem_MLTest &testInfo) = 0;
};

} //End MaceCore Namespace

#endif // I_GROUND_STATION_EVENTS_H
