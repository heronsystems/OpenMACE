#ifndef I_ML_STATION_EVENTS_H
#define I_ML_STATION_EVENTS_H

#include "i_module_events_general.h"

#include "i_module_events_boundary_generator.h"

#include "i_module_events_path_planning.h"

namespace MaceCore
{

class IModuleEventsMLStation : virtual public IModuleEventsPathPlanning, virtual public IModuleEventsGeneral, virtual public IModuleEventsBoundaryGenerator
{
public:
    //!
    //! \brief RequestDummyFunction Test function
    //! \param sender Sender module
    //! \param vehicleID Vehicle ID
    //!
    virtual void RequestDummyFunction(const void* sender, const int &vehicleID) = 0;

    //!
    //! \brief Event_StartRound Event to trigger and new testing round
    //! \param sender Sender module
    //! \param command test setup information
    //!
    virtual void Event_StartRound(const ModuleBase* sender, const DataGenericItem::DataGenericItem_MLTest &testInfo) = 0;

    //!
    //! \brief Event_EndRound Event to stop a running testing round
    //! \param sender Sender module
    //!
    virtual void Event_EndRound(const ModuleBase* sender) = 0;

    //!
    //! \brief Event_MarkTime Event to mark a timestamp on a test log for a particular vehicle
    //! \param sender Sender module
    //! \param command vehicle and timestamp
    //!
    virtual void Event_MarkTime(const ModuleBase* sender, std::string vehicleID, const std::string &timestamp) = 0;
};

} //End MaceCore Namespace

#endif // I_ML_STATION_EVENTS_H
