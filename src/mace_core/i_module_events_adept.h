#ifndef I_MODULE_EVENTS_ADEPT_H
#define I_MODULE_EVENTS_ADEPT_H

#include "i_module_events_general.h"
#include "i_module_events_ai_support.h"

namespace MaceCore
{

class IModuleEventsAdept : virtual public IModuleEventsGeneral, virtual public IModuleEvents_AISupport
{
public:

};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_Adept_H
