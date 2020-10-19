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

};

} //End MaceCore Namespace

#endif // I_ML_STATION_EVENTS_H
