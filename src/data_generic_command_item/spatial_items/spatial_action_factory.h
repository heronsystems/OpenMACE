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
   static AbstractSpatialActionPtr constructFromGoToCommand(const mace_command_goto_t &msg)
   {
       switch (static_cast<COMMANDTYPE>(msg.action)) {
       case COMMANDTYPE::CI_NAV_LOITER_TIME:
       {
            SpatialLoiter_TimePtr action = std::make_shared<SpatialLoiter_Time>();
            //action->fromGoToCommand(msg);
            return action;
           break;
       }
       case COMMANDTYPE::CI_NAV_LOITER_TURNS:
       {
            SpatialLoiter_TurnsPtr action = std::make_shared<SpatialLoiter_Turns>();
            //action->fromGoToCommand(msg);
            return action;
           break;
       }
       case COMMANDTYPE::CI_NAV_LOITER_UNLIM:
       {
            SpatialLoiter_UnlimitedPtr action = std::make_shared<SpatialLoiter_Unlimited>();
            //action->fromGoToCommand(msg);
            return action;
           break;
       }
       case COMMANDTYPE::CI_NAV_WAYPOINT:
       {
            SpatialWaypointPtr action = std::make_shared<SpatialWaypoint>();
            //action->fromGoToCommand(msg);
            return action;
           break;
       }
       default:
           break;
       }
   }
};

} //end of namespace CommandItem
#endif // SPATIAL_ACTION_FACTORY_H
