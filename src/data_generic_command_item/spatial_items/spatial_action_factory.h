#ifndef SPATIAL_ACTION_FACTORY_H
#define SPATIAL_ACTION_FACTORY_H

#include "mace.h"

#include "spatial_components.h"

namespace command_item {

class SpatialActionFactory
{
public:
   SpatialActionFactory() = default;

public:
   static AbstractSpatialActionPtr constructFrom_ExecuteSpatialAction(const mace_execute_spatial_action_t &msg)
   {
       AbstractSpatialActionPtr action;

       switch (static_cast<COMMANDTYPE>(msg.action)) {
       case COMMANDTYPE::CI_NAV_LOITER_TIME:
       {
            action = std::make_shared<SpatialLoiter_Time>();
            //action->fromGoToCommand(msg);
           break;
       }
       case COMMANDTYPE::CI_NAV_LOITER_TURNS:
       {
            action = std::make_shared<SpatialLoiter_Turns>();
            //action->fromGoToCommand(msg);
           break;
       }
       case COMMANDTYPE::CI_NAV_LOITER_UNLIM:
       {
            action = std::make_shared<SpatialLoiter_Unlimited>();
            //action->fromGoToCommand(msg);
           break;
       }
       case COMMANDTYPE::CI_NAV_WAYPOINT:
       {
            action = std::make_shared<SpatialWaypoint>();
            //action->fromGoToCommand(msg);
           break;
       }
       default:
           break;
       }

       return action;
   }
};

} //end of namespace CommandItem
#endif // SPATIAL_ACTION_FACTORY_H
