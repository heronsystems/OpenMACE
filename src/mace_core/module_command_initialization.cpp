#include "i_module_command_path_planning.h"
#include "i_module_command_ROS.h"
#include "i_module_command_RTA.h"
#include "i_module_command_vehicle.h"
#include "i_module_command_ground_station.h"
#include "i_module_command_external_link.h"
#include "i_module_command_sensors.h"

namespace MaceCore
{

ModuleClasses IModuleCommandPathPlanning::moduleClass = ModuleClasses::PATH_PLANNING;
ModuleClasses IModuleCommandROS::moduleClass = ModuleClasses::ROS;
ModuleClasses IModuleCommandRTA::moduleClass = ModuleClasses::RTA;
ModuleClasses IModuleCommandVehicle::moduleClass = ModuleClasses::VEHICLE_COMMS;
ModuleClasses IModuleCommandGroundStation::moduleClass = ModuleClasses::GROUND_STATION;
ModuleClasses IModuleCommandExternalLink::moduleClass = ModuleClasses::EXTERNAL_LINK;
ModuleClasses IModuleCommandSensors::moduleClass = ModuleClasses::SENSORS;

}
