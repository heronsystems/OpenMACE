#ifndef I_RTA_EVENTS_H
#define I_RTA_EVENTS_H

#include "i_module_events_general.h"
#include "i_module_events_boundary_generator.h"

#include "module_characteristics.h"


namespace MaceCore
{

class IModuleEventsRTA  : public IModuleEventsBoundaryGenerator, public IModuleEventsGeneral
{
public:

};

} //End MaceCore Namespace

#endif // I_RTA_EVENTS_H
