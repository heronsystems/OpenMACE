TEMPLATE = subdirs
​
SUBDIRS += \
    digi_common \
    DigiMesh\
    MACEDigiMeshWrapper \
    common \
    base \
    data \
    graphs \
    maps \
    planners \
    comms \
    commsMACE \
    data_generic_item \
    data_generic_command_item \
    mace_core \
    commsMAVLINK \
    commsMACEHelper \
    base_topic \
    data_generic_item_topic \
    data_generic_command_item_topic \
    data_generic_mission_item_topic \
    data_interface_MAVLINK \
    data_vehicle_sensors \
    controllers \
    module_vehicle_generic \
    module_vehicle_MAVLINK \
    module_vehicle_ardupilot \
    module_vehicle_arducopter \
    module_vehicle_arduplane \
    module_external_link\
    module_ground_station \
    module_path_planning_NASAPhase2 \
    module_vehicle_sensors \
    module_resource_task_allocation \
    module_ROS \
    module_ROS_UMD \
    mace \
    trajectory_control

​data.depends = common
DigiMesh.depends = common data digi_common
MACEDigiMeshWrapper.depends = DigiMesh
base.depends = common data
trajectory_control.depends = common data base
graphs.depends = common
maps.depends = common base data
planners.depends = common base maps
comms.depends = common
commsMACE.depends = common data MACEDigiMeshWrapper
data_generic_item.depends = common base data
data_generic_command_item.depends = common base data
mace_core.depends = common base data data_generic_item data_generic_command_item maps
commsMAVLINK.depends = common data comms mace_core
commsMACEHelper.depends = common data commsMACE MACEDigiMeshWrapper mace_core
base_topic.depends = common base data
data_generic_item_topic.depends = common base data data_generic_item
data_generic_command_item_topic.depends = data_generic_command_item
data_generic_mission_item_topic.depends = common base data data_generic_command_item_topic
data_interface_MAVLINK.depends = common base data comms mace_core data_generic_item data_generic_command_item data_generic_item_topic
data_vehicle_sensors.depends = base mace_core
controllers.depends = common mace_core
module_vehicle_generic.depends = common comms data mace_core data_generic_item data_generic_command_item data_generic_item_topic data_generic_command_item_topic data_generic_mission_item_topic trajectory_control
module_vehicle_MAVLINK.depends = base base_topic data comms commsMAVLINK mace_core controllers data_generic_item data_generic_item_topic data_generic_command_item data_generic_command_item_topic data_generic_mission_item_topic module_vehicle_generic
module_vehicle_ardupilot.depends = base base_topic data mace_core comms commsMAVLINK data_generic_item data_generic_item_topic data_generic_command_item data_generic_command_item_topic data_generic_mission_item_topic module_vehicle_generic module_vehicle_generic module_vehicle_MAVLINK data_interface_MAVLINK base_topic
module_vehicle_arducopter.depends = base base_topic data mace_core comms commsMAVLINK data_generic_item data_generic_item_topic data_generic_command_item data_generic_command_item_topic data_generic_mission_item_topic module_vehicle_generic module_vehicle_generic module_vehicle_MAVLINK data_interface_MAVLINK base_topic module_vehicle_ardupilot
module_vehicle_arduplane.depends = base base_topic data mace_core comms commsMAVLINK data_generic_item data_generic_item_topic data_generic_command_item data_generic_command_item_topic data_generic_mission_item_topic module_vehicle_generic module_vehicle_generic module_vehicle_MAVLINK data_interface_MAVLINK base_topic module_vehicle_ardupilot
module_external_link.depends = MACEDigiMeshWrapper common data base base_topic commsMACE commsMACEHelper mace_core data_generic_item data_generic_item_topic data_generic_command_item data_generic_command_item_topic data_generic_mission_item_topic
module_ground_station.depends = base mace_core comms data data_vehicle_sensors data_generic_item data_generic_item_topic data_generic_command_item data_generic_command_item_topic data_generic_mission_item_topic base_topic
module_path_planning_NASAPhase2.depends = common data base base_topic mace_core data_generic_item_topic data_generic_command_item data_generic_command_item_topic data_generic_mission_item_topic maps planners
module_vehicle_sensors.depends = common base data comms mace_core data_generic_item data_generic_item_topic data_generic_command_item data_generic_command_item_topic data_generic_mission_item_topic data_vehicle_sensors maps
module_resource_task_allocation.depends = data base base_topic maps mace_core comms data_generic_item data_generic_item_topic data_generic_command_item_topic data_generic_mission_item_topic data_vehicle_sensors
module_ROS.depends = common base data mace_core base_topic data_generic_item data_generic_item_topic data_generic_command_item maps
module_ROS_UMD.depends = common base data mace_core base_topic data_generic_item data_generic_item_topic data_generic_command_item data_generic_mission_item_topic maps
module_vehicle_adept.depends = common base data comms mace_core data_generic_item data_generic_item_topic data_generic_command_item data_generic_command_item_topic data_generic_mission_item_topic data_vehicle_sensors maps

mace.depends = common data base base_topic maps comms commsMACE commsMACEHelper commsMAVLINK controllers data_generic_command_item data_generic_command_item_topic data_generic_item data_generic_item_topic data_generic_mission_item_topic data_interface_MAVLINK data_vehicle_sensors mace_core module_external_link module_ground_station module_resource_task_allocation module_path_planning_NASAPhase2 module_ROS module_ROS_UMD module_vehicle_ardupilot module_vehicle_arducopter module_vehicle_arduplane module_vehicle_generic module_vehicle_MAVLINK module_vehicle_sensors planners DigiMesh MACEDigiMeshWrapper
