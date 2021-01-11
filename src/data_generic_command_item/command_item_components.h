#ifndef COMMAND_ITEM_COMPONENTS_H
#define COMMAND_ITEM_COMPONENTS_H

#include "data_generic_command_item/mace/ai_items/ai_command_components.h"

#include "data_generic_command_item/boundary_items/boundary_key.h"
#include "data_generic_command_item/boundary_items/boundary_type.h"
#include "data_generic_command_item/boundary_items/boundary_list.h"
#include "data_generic_command_item/boundary_items/environment_boundary.h"

#include "data_generic_command_item/spatial_items/abstract_spatial_action.h"
#include "data_generic_command_item/spatial_items/spatial_home.h"
#include "data_generic_command_item/spatial_items/spatial_loiter_time.h"
#include "data_generic_command_item/spatial_items/spatial_loiter_turns.h"
#include "data_generic_command_item/spatial_items/spatial_loiter_unlimited.h"
#include "data_generic_command_item/spatial_items/spatial_land.h"
#include "data_generic_command_item/spatial_items/spatial_rtl.h"
#include "data_generic_command_item/spatial_items/spatial_takeoff.h"
#include "data_generic_command_item/spatial_items/spatial_waypoint.h"

#include "data_generic_command_item/do_items/action_arm.h"
#include "data_generic_command_item/do_items/action_change_mode.h"
#include "data_generic_command_item/do_items/action_change_speed.h"
#include "data_generic_command_item/do_items/action_dynamic_target.h"
#include "data_generic_command_item/do_items/action_execute_spatial_item.h"
#include "data_generic_command_item/do_items/action_message_interval.h"
#include "data_generic_command_item/do_items/action_message_request.h"
#include "data_generic_command_item/do_items/action_mission_command.h"
#include "data_generic_command_item/do_items/action_motor_test.h"
#include "data_generic_command_item/do_items/action_set_global_origin.h"
#include "data_generic_command_item/do_items/action_set_surface_deflection.h"

#include "data_generic_command_item/mission_items/mission_ack.h"
#include "data_generic_command_item/mission_items/mission_item_achieved.h"
#include "data_generic_command_item/mission_items/mission_item_current.h"
#include "data_generic_command_item/mission_items/mission_key.h"
#include "data_generic_command_item/mission_items/mission_key_change.h"
#include "data_generic_command_item/mission_items/mission_list.h"
#include "data_generic_command_item/mission_items/mission_state.h"
#include "data_generic_command_item/mission_items/typedef_mission_types.h"

#include "data_generic_command_item/target_items/dynamic_mission_queue.h"
#include "data_generic_command_item/target_items/dynamic_target_kinematic.h"
#include "data_generic_command_item/target_items/dynamic_target_list.h"
#include "data_generic_command_item/target_items/dynamic_target_state.h"

#include "command_item_type.h"

#endif // COMMAND_ITEM_COMPONENTS_H
