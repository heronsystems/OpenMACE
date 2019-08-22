/** @file
 *	@brief MAVLink comm protocol generated from common.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace common {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (trought @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 52> MESSAGE_ENTRIES {{ {0, 39, 7, 0, 0, 0}, {1, 204, 10, 0, 0, 0}, {2, 34, 1, 0, 0, 0}, {3, 227, 10, 0, 0, 0}, {4, 137, 12, 0, 0, 0}, {5, 237, 14, 3, 12, 13}, {6, 217, 28, 1, 0, 0}, {7, 104, 3, 0, 0, 0}, {8, 119, 32, 0, 0, 0}, {9, 214, 20, 3, 2, 3}, {10, 159, 2, 3, 0, 1}, {11, 220, 25, 0, 0, 0}, {12, 168, 23, 3, 4, 5}, {13, 24, 30, 0, 0, 0}, {14, 23, 101, 0, 0, 0}, {15, 115, 14, 0, 0, 0}, {16, 61, 12, 0, 0, 0}, {17, 239, 12, 0, 0, 0}, {18, 251, 24, 0, 0, 0}, {19, 246, 32, 0, 0, 0}, {20, 121, 16, 0, 0, 0}, {21, 62, 16, 0, 0, 0}, {22, 57, 28, 0, 0, 0}, {23, 187, 22, 0, 0, 0}, {24, 245, 10, 0, 0, 0}, {25, 128, 28, 0, 0, 0}, {26, 41, 13, 1, 12, 0}, {27, 39, 12, 0, 0, 0}, {28, 20, 20, 0, 0, 0}, {29, 158, 35, 3, 30, 31}, {30, 152, 33, 3, 30, 31}, {31, 177, 9, 3, 6, 7}, {32, 143, 3, 0, 0, 0}, {33, 129, 21, 1, 0, 0}, {34, 1, 1, 0, 0, 0}, {35, 48, 36, 3, 32, 33}, {36, 108, 1, 0, 0, 0}, {200, 185, 9, 0, 0, 0}, {201, 34, 16, 0, 0, 0}, {202, 203, 6, 0, 0, 0}, {203, 85, 14, 0, 0, 0}, {204, 47, 32, 0, 0, 0}, {244, 95, 6, 0, 0, 0}, {245, 130, 2, 0, 0, 0}, {246, 184, 38, 0, 0, 0}, {247, 81, 19, 0, 0, 0}, {253, 83, 51, 0, 0, 0}, {259, 122, 86, 0, 0, 0}, {260, 8, 28, 0, 0, 0}, {262, 69, 31, 0, 0, 0}, {263, 133, 255, 0, 0, 0}, {264, 49, 28, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 3;


// ENUM DEFINITIONS


/** @brief Micro air vehicle / autopilot classes. This identifies the individual model. */
enum class UXV_PROTOCOL
{
    MACE=0, /* MACE protocol | */
    PROTCOL_MAVLINK=1, /* MAVLINK protocol. | */
    DJI=2, /* DJI protocol | */
    UNKNOWN=3, /* Generic autopilot, full support for everything | */
};

//! UXV_PROTOCOL ENUM_END
constexpr auto UXV_PROTOCOL_ENUM_END = 4;

/** @brief Micro air vehicle / autopilot classes. This identifies the individual model. */
enum class UXV_AUTOPILOT : uint8_t
{
    GENERIC=0, /* Generic autopilot, full support for everything | */
    RESERVED=1, /* Reserved for future use. | */
    SLUGS=2, /* SLUGS autopilot, http://slugsuav.soe.ucsc.edu | */
    ARDUPILOTMEGA=3, /* ArduPilotMega / ArduCopter, http://diydrones.com | */
    OPENPILOT=4, /* OpenPilot, http://openpilot.org | */
    GENERIC_WAYPOINTS_ONLY=5, /* Generic autopilot only supporting simple waypoints | */
    GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY=6, /* Generic autopilot supporting waypoints and other simple navigation commands | */
    GENERIC_MISSION_FULL=7, /* Generic autopilot supporting the full mission command set | */
    INVALID=8, /* No valid autopilot, e.g. a GCS or other MAVLink component | */
    PPZ=9, /* PPZ UAV - http://nongnu.org/paparazzi | */
    UDB=10, /* UAV Dev Board | */
    FP=11, /* FlexiPilot | */
    PX4=12, /* PX4 Autopilot - http://pixhawk.ethz.ch/px4/ | */
    SMACCMPILOT=13, /* SMACCMPilot - http://smaccmpilot.org | */
    AUTOQUAD=14, /* AutoQuad -- http://autoquad.org | */
    ARMAZILA=15, /* Armazila -- http://armazila.com | */
    AEROB=16, /* Aerob -- http://aerob.ru | */
    ASLUAV=17, /* ASLUAV autopilot -- http://www.asl.ethz.ch | */
    SMARTAP=18, /* SmartAP Autopilot - http://sky-drones.com | */
    DJI=19, /* DJI Autopilot | */
};

//! UXV_AUTOPILOT ENUM_END
constexpr auto UXV_AUTOPILOT_ENUM_END = 20;

/** @brief  */
enum class UXV_TYPE : uint8_t
{
    GENERIC=0, /* Generic micro air vehicle. | */
    FIXED_WING=1, /* Fixed wing aircraft. | */
    QUADROTOR=2, /* Quadrotor | */
    COAXIAL=3, /* Coaxial helicopter | */
    HELICOPTER=4, /* Normal helicopter with tail rotor. | */
    ANTENNA_TRACKER=5, /* Ground installation | */
    GCS=6, /* Operator control unit / ground control station | */
    AIRSHIP=7, /* Airship, controlled | */
    FREE_BALLOON=8, /* Free balloon, uncontrolled | */
    ROCKET=9, /* Rocket | */
    GROUND_ROVER=10, /* Ground rover | */
    SURFACE_BOAT=11, /* Surface vessel, boat, ship | */
    SUBMARINE=12, /* Submarine | */
    HEXAROTOR=13, /* Hexarotor | */
    OCTOROTOR=14, /* Octorotor | */
    TRICOPTER=15, /* Tricopter | */
    FLAPPING_WING=16, /* Flapping wing | */
    KITE=17, /* Kite | */
    ONBOARD_CONTROLLER=18, /* Onboard companion controller | */
    VTOL_DUOROTOR=19, /* Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter. | */
    VTOL_QUADROTOR=20, /* Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter. | */
    VTOL_TILTROTOR=21, /* Tiltrotor VTOL | */
    VTOL_RESERVED2=22, /* VTOL reserved 2 | */
    VTOL_RESERVED3=23, /* VTOL reserved 3 | */
    VTOL_RESERVED4=24, /* VTOL reserved 4 | */
    VTOL_RESERVED5=25, /* VTOL reserved 5 | */
    GIMBAL=26, /* Onboard gimbal | */
    ADSB=27, /* Onboard ADSB peripheral | */
};

//! UXV_TYPE ENUM_END
constexpr auto UXV_TYPE_ENUM_END = 28;

/** @brief These values define the type of firmware release.  These values indicate the first version or release of this type.  For example the first alpha release would be 64, the second would be 65. */
enum class FIRMWARE_VERSION_TYPE
{
    DEV=0, /* development release | */
    ALPHA=64, /* alpha release | */
    BETA=128, /* beta release | */
    RC=192, /* release candidate | */
    OFFICIAL=255, /* official stable release | */
};

//! FIRMWARE_VERSION_TYPE ENUM_END
constexpr auto FIRMWARE_VERSION_TYPE_ENUM_END = 256;

/** @brief These flags encode the MAV mode. */
enum class UXV_MODE_FLAG
{
    CUSTOM_MODE_ENABLED=1, /* 0b00000001 Reserved for future use. | */
    TEST_ENABLED=2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
    AUTO_ENABLED=4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
    GUIDED_ENABLED=8, /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
    STABILIZE_ENABLED=16, /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
    HIL_ENABLED=32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
    MANUAL_INPUT_ENABLED=64, /* 0b01000000 remote control input is enabled. | */
    SAFETY_ARMED=128, /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command UXV_CMD_DO_SET_MODE and UXV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state. | */
};

//! UXV_MODE_FLAG ENUM_END
constexpr auto UXV_MODE_FLAG_ENUM_END = 129;

/** @brief These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not. */
enum class UXV_MODE_FLAG_DECODE_POSITION
{
    CUSTOM_MODE=1, /* Eighth bit: 00000001 | */
    TEST=2, /* Seventh bit: 00000010 | */
    AUTO=4, /* Sixt bit:   00000100 | */
    GUIDED=8, /* Fifth bit:  00001000 | */
    STABILIZE=16, /* Fourth bit: 00010000 | */
    HIL=32, /* Third bit:  00100000 | */
    MANUAL=64, /* Second bit: 01000000 | */
    SAFETY=128, /* First bit:  10000000 | */
};

//! UXV_MODE_FLAG_DECODE_POSITION ENUM_END
constexpr auto UXV_MODE_FLAG_DECODE_POSITION_ENUM_END = 129;

/** @brief Override command, pauses current mission execution and moves immediately to a position */
enum class UXV_GOTO
{
    DO_HOLD=0, /* Hold at the current position. | */
    DO_CONTINUE=1, /* Continue with the next item in mission execution. | */
    HOLD_AT_CURRENT_POSITION=2, /* Hold at the current position of the system | */
    HOLD_AT_SPECIFIED_POSITION=3, /* Hold at the position specified in the parameters of the DO_HOLD action | */
};

//! UXV_GOTO ENUM_END
constexpr auto UXV_GOTO_ENUM_END = 4;

/** @brief These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
               simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override. */
enum class UXV_MODE
{
    PREFLIGHT=0, /* System is not ready to fly, booting, calibrating, etc. No flag is set. | */
    MANUAL_DISARMED=64, /* System is allowed to be active, under manual (RC) control, no stabilization | */
    TEST_DISARMED=66, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
    STABILIZE_DISARMED=80, /* System is allowed to be active, under assisted RC control. | */
    GUIDED_DISARMED=88, /* System is allowed to be active, under autonomous control, manual setpoint | */
    AUTO_DISARMED=92, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) | */
    MANUAL_ARMED=192, /* System is allowed to be active, under manual (RC) control, no stabilization | */
    TEST_ARMED=194, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
    STABILIZE_ARMED=208, /* System is allowed to be active, under assisted RC control. | */
    GUIDED_ARMED=216, /* System is allowed to be active, under autonomous control, manual setpoint | */
    AUTO_ARMED=220, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) | */
};

//! UXV_MODE ENUM_END
constexpr auto UXV_MODE_ENUM_END = 221;

/** @brief  */
enum class UXV_STATE
{
    UNINIT=0, /* Uninitialized system, state is unknown. | */
    BOOT=1, /* System is booting up. | */
    CALIBRATING=2, /* System is calibrating and not flight-ready. | */
    STANDBY=3, /* System is grounded and on standby. It can be launched any time. | */
    ACTIVE=4, /* System is active and might be already airborne. Motors are engaged. | */
    CRITICAL=5, /* System is in a non-normal flight mode. It can however still navigate. | */
    EMERGENCY=6, /* System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | */
    POWEROFF=7, /* System just initialized its power-down sequence, will shut down now. | */
};

//! UXV_STATE ENUM_END
constexpr auto UXV_STATE_ENUM_END = 8;

/** @brief  */
enum class UXV_COMPONENT
{
    COMP_ID_ALL=0, /*  | */
    COMP_ID_CAMERA=100, /*  | */
    COMP_ID_SERVO1=140, /*  | */
    COMP_ID_SERVO2=141, /*  | */
    COMP_ID_SERVO3=142, /*  | */
    COMP_ID_SERVO4=143, /*  | */
    COMP_ID_SERVO5=144, /*  | */
    COMP_ID_SERVO6=145, /*  | */
    COMP_ID_SERVO7=146, /*  | */
    COMP_ID_SERVO8=147, /*  | */
    COMP_ID_SERVO9=148, /*  | */
    COMP_ID_SERVO10=149, /*  | */
    COMP_ID_SERVO11=150, /*  | */
    COMP_ID_SERVO12=151, /*  | */
    COMP_ID_SERVO13=152, /*  | */
    COMP_ID_SERVO14=153, /*  | */
    COMP_ID_GIMBAL=154, /*  | */
    COMP_ID_LOG=155, /*  | */
    COMP_ID_ADSB=156, /*  | */
    COMP_ID_OSD=157, /* On Screen Display (OSD) devices for video links | */
    COMP_ID_PERIPHERAL=158, /* Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter sub-protocol | */
    COMP_ID_QX1_GIMBAL=159, /*  | */
    COMP_ID_MAPPER=180, /*  | */
    COMP_ID_MISSIONPLANNER=190, /*  | */
    COMP_ID_PATHPLANNER=195, /*  | */
    COMP_ID_IMU=200, /*  | */
    COMP_ID_IMU_2=201, /*  | */
    COMP_ID_IMU_3=202, /*  | */
    COMP_ID_GPS=220, /*  | */
    COMP_ID_UDP_BRIDGE=240, /*  | */
    COMP_ID_UART_BRIDGE=241, /*  | */
    COMP_ID_SYSTEM_CONTROL=250, /*  | */
};

//! UXV_COMPONENT ENUM_END
constexpr auto UXV_COMPONENT_ENUM_END = 251;

/** @brief These encode the sensors whose status is sent as part of the SYS_STATUS message. */
enum class UXV_SYS_STATUS_SENSOR
{
    SENSOR_3D_GYRO=1, /* 0x01 3D gyro | */
    SENSOR_3D_ACCEL=2, /* 0x02 3D accelerometer | */
    SENSOR_3D_MAG=4, /* 0x04 3D magnetometer | */
    ABSOLUTE_PRESSURE=8, /* 0x08 absolute pressure | */
    DIFFERENTIAL_PRESSURE=16, /* 0x10 differential pressure | */
    GPS=32, /* 0x20 GPS | */
    OPTICAL_FLOW=64, /* 0x40 optical flow | */
    VISION_POSITION=128, /* 0x80 computer vision position | */
    LASER_POSITION=256, /* 0x100 laser based position | */
    EXTERNAL_GROUND_TRUTH=512, /* 0x200 external ground truth (Vicon or Leica) | */
    ANGULAR_RATE_CONTROL=1024, /* 0x400 3D angular rate control | */
    ATTITUDE_STABILIZATION=2048, /* 0x800 attitude stabilization | */
    YAW_POSITION=4096, /* 0x1000 yaw position | */
    Z_ALTITUDE_CONTROL=8192, /* 0x2000 z/altitude control | */
    XY_POSITION_CONTROL=16384, /* 0x4000 x/y position control | */
    MOTOR_OUTPUTS=32768, /* 0x8000 motor outputs / control | */
    RC_RECEIVER=65536, /* 0x10000 rc receiver | */
    SENSOR_3D_GYRO2=131072, /* 0x20000 2nd 3D gyro | */
    SENSOR_3D_ACCEL2=262144, /* 0x40000 2nd 3D accelerometer | */
    SENSOR_3D_MAG2=524288, /* 0x80000 2nd 3D magnetometer | */
    GEOFENCE=1048576, /* 0x100000 geofence | */
    AHRS=2097152, /* 0x200000 AHRS subsystem health | */
    TERRAIN=4194304, /* 0x400000 Terrain subsystem health | */
    REVERSE_MOTOR=8388608, /* 0x800000 Motors are reversed | */
    LOGGING=16777216, /* 0x1000000 Logging | */
    BATTERY=33554432, /* 0x2000000 Battery | */
};

//! UXV_SYS_STATUS_SENSOR ENUM_END
constexpr auto UXV_SYS_STATUS_SENSOR_ENUM_END = 33554433;

/** @brief  */
enum class UXV_FRAME : uint8_t
{
    GLOBAL=0, /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL) | */
    LOCAL_NED=1, /* Local coordinate frame, Z-up (x: north, y: east, z: down). | */
    MISSION=2, /* NOT a coordinate frame, indicates a mission command. | */
    GLOBAL_RELATIVE_ALT=3, /* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. | */
    LOCAL_ENU=4, /* Local coordinate frame, Z-down (x: east, y: north, z: up) | */
    GLOBAL_INT=5, /* Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL) | */
    GLOBAL_RELATIVE_ALT_INT=6, /* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location. | */
    LOCAL_OFFSET_NED=7, /* Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position. | */
    BODY_NED=8, /* Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right. | */
    BODY_OFFSET_NED=9, /* Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east. | */
    GLOBAL_TERRAIN_ALT=10, /* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */
    GLOBAL_TERRAIN_ALT_INT=11, /* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */
};

//! UXV_FRAME ENUM_END
constexpr auto UXV_FRAME_ENUM_END = 12;

/** @brief  */
enum class MAVLINK_DATA_STREAM_TYPE
{
    IMG_JPEG=1, /*  | */
    IMG_BMP=2, /*  | */
    IMG_RAW8U=3, /*  | */
    IMG_RAW32U=4, /*  | */
    IMG_PGM=5, /*  | */
    IMG_PNG=6, /*  | */
};

//! MAVLINK_DATA_STREAM_TYPE ENUM_END
constexpr auto MAVLINK_DATA_STREAM_TYPE_ENUM_END = 7;

/** @brief  */
enum class FENCE_ACTION
{
    NONE=0, /* Disable fenced mode | */
    GUIDED=1, /* Switched to guided mode to return point (fence point 0) | */
    REPORT=2, /* Report fence breach, but don't take action | */
    GUIDED_THR_PASS=3, /* Switched to guided mode to return point (fence point 0) with manual throttle control | */
    RTL=4, /* Switch to RTL (return to launch) mode and head for the return point. | */
};

//! FENCE_ACTION ENUM_END
constexpr auto FENCE_ACTION_ENUM_END = 5;

/** @brief  */
enum class FENCE_BREACH
{
    NONE=0, /* No last fence breach | */
    MINALT=1, /* Breached minimum altitude | */
    MAXALT=2, /* Breached maximum altitude | */
    BOUNDARY=3, /* Breached fence boundary | */
};

//! FENCE_BREACH ENUM_END
constexpr auto FENCE_BREACH_ENUM_END = 4;

/** @brief Enumeration of possible mount operation modes */
enum class UXV_MOUNT_MODE
{
    RETRACT=0, /* Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization | */
    NEUTRAL=1, /* Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory. | */
    MAVLINK_TARGETING=2, /* Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization | */
    RC_TARGETING=3, /* Load neutral position and start RC Roll,Pitch,Yaw control with stabilization | */
    GPS_POINT=4, /* Load neutral position and start to point to Lat,Lon,Alt | */
};

//! UXV_MOUNT_MODE ENUM_END
constexpr auto UXV_MOUNT_MODE_ENUM_END = 5;

/** @brief Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. */
enum class UXV_CMD : uint16_t
{
    NAV_WAYPOINT=16, /* Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing). NaN for unchanged.| Latitude| Longitude| Altitude|  */
    NAV_LOITER_UNLIM=17, /* Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
    NAV_LOITER_TURNS=18, /* Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  */
    NAV_LOITER_TIME=19, /* Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location. Else, this is desired yaw angle| Latitude| Longitude| Altitude|  */
    NAV_RETURN_TO_LAUNCH=20, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    NAV_LAND=21, /* Land at location |Abort Alt| Empty| Empty| Desired yaw angle. NaN for unchanged.| Latitude| Longitude| Altitude|  */
    NAV_TAKEOFF=22, /* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer. NaN for unchanged.| Latitude| Longitude| Altitude|  */
    NAV_LAND_LOCAL=23, /* Land at local position (local frame only) |Landing target number (if available)| Maximum accepted offset from desired landing position [m] - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land| Landing descend rate [ms^-1]| Desired yaw angle [rad]| Y-axis position [m]| X-axis position [m]| Z-axis / ground level position [m]|  */
    NAV_TAKEOFF_LOCAL=24, /* Takeoff from local position (local frame only) |Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]| Empty| Takeoff ascend rate [ms^-1]| Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these| Y-axis position [m]| X-axis position [m]| Z-axis position [m]|  */
    NAV_FOLLOW=25, /* Vehicle following, i.e. this waypoint represents the position of a moving vehicle |Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation| Ground speed of vehicle to be followed| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
    NAV_CONTINUE_AND_CHANGE_ALT=30, /* Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. |Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude. | Empty| Empty| Empty| Empty| Empty| Desired altitude in meters|  */
    NAV_LOITER_TO_ALT=31, /* Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint.  |Heading Required (0 = False)| Radius in meters. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.| Empty| Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location| Latitude| Longitude| Altitude|  */
    DO_FOLLOW=32, /* Being following a target |System ID (the system ID of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode| RESERVED| RESERVED| altitude flag: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home| altitude| RESERVED| TTL in seconds in which the MAV should go to the default position hold mode after a message rx timeout|  */
    DO_FOLLOW_REPOSITION=33, /* Reposition the MAV after a follow target command has been sent |Camera q1 (where 0 is on the ray from the camera to the tracking device)| Camera q2| Camera q3| Camera q4| altitude offset from target (m)| X offset from target (m)| Y offset from target (m)|  */
    NAV_ROI=80, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see UXV_ROI enum)| MISSION index/ target ID. (see UXV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see UXV_FRAME)| y| z|  */
    NAV_PATHPLANNING=81, /* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
    NAV_SPLINE_WAYPOINT=82, /* Navigate to MISSION using a spline path. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Empty| Empty| Empty| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
    NAV_VTOL_TAKEOFF=84, /* Takeoff from ground using VTOL mode |Empty| Empty| Empty| Yaw angle in degrees. NaN for unchanged.| Latitude| Longitude| Altitude|  */
    NAV_VTOL_LAND=85, /* Land using VTOL mode |Empty| Empty| Empty| Yaw angle in degrees. NaN for unchanged.| Latitude| Longitude| Altitude|  */
    NAV_GUIDED_ENABLE=92, /* hand control over to an external controller |On / Off (> 0.5f on)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    NAV_DELAY=93, /* Delay the next navigation command a number of seconds or until a specified time |Delay in seconds (decimal, -1 to enable time-of-day fields)| hour (24h format, UTC, -1 to ignore)| minute (24h format, UTC, -1 to ignore)| second (24h format, UTC)| Empty| Empty| Empty|  */
    NAV_PAYLOAD_PLACE=94, /* Descend and place payload.  Vehicle descends until it detects a hanging payload has reached the ground, the gripper is opened to release the payload |Maximum distance to descend (meters)| Empty| Empty| Empty| Latitude (deg * 1E7)| Longitude (deg * 1E7)| Altitude (meters)|  */
    NAV_LAST=95, /* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    CONDITION_DELAY=112, /* Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    CONDITION_CHANGE_ALT=113, /* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  */
    CONDITION_DISTANCE=114, /* Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    CONDITION_YAW=115, /* Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  */
    CONDITION_LAST=159, /* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_SET_MODE=176, /* Set system mode. |Mode, as defined by ENUM UXV_MODE| Custom mode - this is system specific, please refer to the individual autopilot specifications for details.| Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.| Empty| Empty| Empty| Empty|  */
    DO_JUMP=177, /* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  */
    DO_CHANGE_SPEED=178, /* Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| absolute or relative [0,1]| Empty| Empty| Empty|  */
    DO_SET_HOME=179, /* Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
    DO_SET_PARAMETER=180, /* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */
    DO_SET_RELAY=181, /* Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  */
    DO_REPEAT_RELAY=182, /* Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  */
    DO_SET_SERVO=183, /* Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  */
    DO_REPEAT_SERVO=184, /* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  */
    DO_FLIGHTTERMINATION=185, /* Terminate flight immediately |Flight termination activated if > 0.5| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_CHANGE_ALTITUDE=186, /* Change altitude set point. |Altitude in meters| Mav frame of new altitude (see UXV_FRAME)| Empty| Empty| Empty| Empty| Empty|  */
    DO_LAND_START=189, /* Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0/0 if not needed. If specified then it will be used to help find the closest landing sequence. |Empty| Empty| Empty| Empty| Latitude| Longitude| Empty|  */
    DO_RALLY_LAND=190, /* Mission command to perform a landing from a rally point. |Break altitude (meters)| Landing speed (m/s)| Empty| Empty| Empty| Empty| Empty|  */
    DO_GO_AROUND=191, /* Mission command to safely abort an autonmous landing. |Altitude (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_REPOSITION=192, /* Reposition the vehicle to a specific WGS84 global position. |Ground speed, less than 0 (-1) for default| Bitmask of option flags, see the UXV_DO_REPOSITION_FLAGS enum.| Reserved| Yaw heading, NaN for unchanged. For planes indicates loiter direction (0: clockwise, 1: counter clockwise)| Latitude (deg * 1E7)| Longitude (deg * 1E7)| Altitude (meters)|  */
    DO_PAUSE_CONTINUE=193, /* If in a GPS controlled position mode, hold the current position or continue. |0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    DO_SET_REVERSE=194, /* Set moving direction to forward or reverse. |Direction (0=Forward, 1=Reverse)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_CONTROL_VIDEO=200, /* Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */
    DO_SET_ROI=201, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see UXV_ROI enum)| MISSION index/ target ID. (see UXV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see UXV_FRAME)| y| z|  */
    DO_DIGICAM_CONFIGURE=202, /* Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|  */
    DO_DIGICAM_CONTROL=203, /* Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|  */
    DO_MOUNT_CONFIGURE=204, /* Mission command to configure a camera or antenna mount |Mount operation mode (see UXV_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|  */
    DO_MOUNT_CONTROL=205, /* Mission command to control a camera or antenna mount |pitch (WIP: DEPRECATED: or lat in degrees) depending on mount mode.| roll (WIP: DEPRECATED: or lon in degrees) depending on mount mode.| yaw (WIP: DEPRECATED: or alt in meters) depending on mount mode.| WIP: alt in meters depending on mount mode.| WIP: latitude in degrees * 1E7, set if appropriate mount mode.| WIP: longitude in degrees * 1E7, set if appropriate mount mode.| UXV_MOUNT_MODE enum value|  */
    DO_SET_CAM_TRIGG_DIST=206, /* Mission command to set CAM_TRIGG_DIST for this flight |Camera trigger distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_FENCE_ENABLE=207, /* Mission command to enable the geofence |enable? (0=disable, 1=enable, 2=disable_floor_only)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_PARACHUTE=208, /* Mission command to trigger a parachute |action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_MOTOR_TEST=209, /* Mission command to perform motor test |motor sequence number (a number from 1 to max number of motors on the vehicle)| throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)| throttle| timeout (in seconds)| Empty| Empty| Empty|  */
    DO_INVERTED_FLIGHT=210, /* Change to/from inverted flight |inverted (0=normal, 1=inverted)| Empty| Empty| Empty| Empty| Empty| Empty|  */
    NAV_SET_YAW_SPEED=213, /* Sets a desired vehicle turn angle and speed change |yaw angle to adjust steering by in centidegress| speed - normalized to 0 .. 1| Empty| Empty| Empty| Empty| Empty|  */
    DO_MOUNT_CONTROL_QUAT=220, /* Mission command to control a camera or antenna mount, using a quaternion as reference. |q1 - quaternion param #1, w (1 in null-rotation)| q2 - quaternion param #2, x (0 in null-rotation)| q3 - quaternion param #3, y (0 in null-rotation)| q4 - quaternion param #4, z (0 in null-rotation)| Empty| Empty| Empty|  */
    DO_GUIDED_MASTER=221, /* set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|  */
    DO_GUIDED_LIMITS=222, /* set limits for external control |timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout| absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit| absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit| horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit| Empty| Empty| Empty|  */
    DO_ENGINE_CONTROL=223, /* Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines |0: Stop engine, 1:Start Engine| 0: Warm start, 1:Cold start. Controls use of choke where applicable| Height delay (meters). This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.| Empty| Empty| Empty| Empty| Empty|  */
    DO_LAST=240, /* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    PREFLIGHT_CALIBRATION=241, /* Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero. |1: gyro calibration, 3: gyro temperature calibration| 1: magnetometer calibration| 1: ground pressure calibration| 1: radio RC calibration, 2: RC trim calibration| 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration| 1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration| 1: ESC calibration, 3: barometer temperature calibration|  */
    PREFLIGHT_SET_SENSOR_OFFSETS=242, /* Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  */
    PREFLIGHT_UAVCAN=243, /* Trigger UAVCAN config. This command will be only accepted if in pre-flight mode. |1: Trigger actuator ID assignment and direction mapping.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    PREFLIGHT_STORAGE=245, /* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: start logging with rate of param 3 in Hz (e.g. set to 1000 for 1000 Hz logging)| Reserved| Empty| Empty| Empty|  */
    PREFLIGHT_REBOOT_SHUTDOWN=246, /* Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.| WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded| WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded| Reserved, send 0| Reserved, send 0| WIP: ID (e.g. camera ID -1 for all IDs)|  */
    OVERRIDE_GOTO=252, /* Hold / continue the current action |UXV_GOTO_DO_HOLD: hold UXV_GOTO_DO_CONTINUE: continue with next item in mission plan| UXV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position UXV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| UXV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  */
    MISSION_START=300, /* start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)|  */
    COMPONENT_ARM_DISARM=400, /* Arms / Disarms a component |1 to arm, 0 to disarm|  */
    GET_HOME_POSITION=410, /* Request the home position from the vehicle. |Reserved| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    START_RX_PAIR=500, /* Starts receiver pairing |0:Spektrum| 0:Spektrum DSM2, 1:Spektrum DSMX|  */
    GET_MESSAGE_INTERVAL=510, /* Request the interval between messages for a particular MAVLink message ID |The MAVLink message ID|  */
    SET_MESSAGE_INTERVAL=511, /* Request the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM |The MAVLink message ID| The interval between two messages, in microseconds. Set to -1 to disable and 0 to request default rate.|  */
    REQUEST_AUTOPILOT_CAPABILITIES=520, /* Request autopilot capabilities |1: Request autopilot version| Reserved (all remaining params)|  */
    REQUEST_CAMERA_INFORMATION=521, /* WIP: Request camera information (CAMERA_INFORMATION) |1: Request camera capabilities| Camera ID| Reserved (all remaining params)|  */
    REQUEST_CAMERA_SETTINGS=522, /* WIP: Request camera settings (CAMERA_SETTINGS) |1: Request camera settings| Camera ID| Reserved (all remaining params)|  */
    SET_CAMERA_SETTINGS_1=523, /* WIP: Set the camera settings part 1 (CAMERA_SETTINGS) |Camera ID| Aperture (1/value)| Aperture locked (0: auto, 1: locked)| Shutter speed in s| Shutter speed locked (0: auto, 1: locked)| ISO sensitivity| ISO sensitivity locked (0: auto, 1: locked)|  */
    SET_CAMERA_SETTINGS_2=524, /* WIP: Set the camera settings part 2 (CAMERA_SETTINGS) |Camera ID| White balance locked (0: auto, 1: locked)| White balance (color temperature in K)| Reserved for camera mode ID| Reserved for color mode ID| Reserved for image format ID| Reserved|  */
    REQUEST_STORAGE_INFORMATION=525, /* WIP: Request storage information (STORAGE_INFORMATION) |1: Request storage information| Storage ID| Reserved (all remaining params)|  */
    STORAGE_FORMAT=526, /* WIP: Format a storage medium |1: Format storage| Storage ID| Reserved (all remaining params)|  */
    REQUEST_CAMERA_CAPTURE_STATUS=527, /* WIP: Request camera capture status (CAMERA_CAPTURE_STATUS) |1: Request camera capture status| Camera ID| Reserved (all remaining params)|  */
    REQUEST_FLIGHT_INFORMATION=528, /* WIP: Request flight information (FLIGHT_INFORMATION) |1: Request flight information| Reserved (all remaining params)|  */
    IMAGE_START_CAPTURE=2000, /* Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. |Duration between two consecutive pictures (in seconds)| Number of images to capture total - 0 for unlimited capture| Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc), set to 0 if param 4/5 are used, set to -1 for highest resolution possible.| WIP: Resolution horizontal in pixels| WIP: Resolution horizontal in pixels| WIP: Camera ID|  */
    IMAGE_STOP_CAPTURE=2001, /* Stop image capture sequence |Camera ID| Reserved|  */
    DO_TRIGGER_CONTROL=2003, /* Enable or disable on-board camera triggering system. |Trigger enable/disable (0 for disable, 1 for start)| Shutter integration time (in ms)| Reserved|  */
    VIDEO_START_CAPTURE=2500, /* Starts video capture (recording) |Camera ID (0 for all cameras), 1 for first, 2 for second, etc.| Frames per second, set to -1 for highest framerate possible.| Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc), set to 0 if param 4/5 are used, set to -1 for highest resolution possible.| WIP: Resolution horizontal in pixels| WIP: Resolution horizontal in pixels| WIP: Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise time in Hz)|  */
    VIDEO_STOP_CAPTURE=2501, /* Stop the current video capture (recording) |WIP: Camera ID| Reserved|  */
    LOGGING_START=2510, /* Request to start streaming logging data over MAVLink (see also LOGGING_DATA message) |Format: 0: ULog| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
    LOGGING_STOP=2511, /* Request to stop streaming log data over MAVLink |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
    AIRFRAME_CONFIGURATION=2520, /*  |Landing gear ID (default: 0, -1 for all)| Landing gear position (Down: 0, Up: 1, NAN for no change)| Reserved, set to NAN| Reserved, set to NAN| Reserved, set to NAN| Reserved, set to NAN| Reserved, set to NAN|  */
    PANORAMA_CREATE=2800, /* Create a panorama at the current position |Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)| Viewing angle vertical of panorama (in degrees)| Speed of the horizontal rotation (in degrees per second)| Speed of the vertical rotation (in degrees per second)|  */
    DO_VTOL_TRANSITION=3000, /* Request VTOL transition |The target VTOL state, as defined by ENUM UXV_VTOL_STATE. Only UXV_VTOL_STATE_MC and UXV_VTOL_STATE_FW can be used.|  */
    SET_GUIDED_SUBMODE_STANDARD=4000, /* This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocites along all three axes.
                   | */
    SET_GUIDED_SUBMODE_CIRCLE=4001, /* This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
                   |Radius of desired circle in CIRCLE_MODE| User defined| User defined| User defined| Unscaled target latitude of center of circle in CIRCLE_MODE| Unscaled target longitude of center of circle in CIRCLE_MODE|  */
    NAV_FENCE_RETURN_POINT=5000, /* Fence return point. There can only be one fence return point.
         |Reserved| Reserved| Reserved| Reserved| Latitude| Longitude| Altitude|  */
    NAV_FENCE_POLYGON_VERTEX_INCLUSION=5001, /* Fence vertex for an inclusion polygon. The vehicle must stay within this area. Minimum of 3 vertices required.
         |Polygon vertex count| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
    NAV_FENCE_POLYGON_VERTEX_EXCLUSION=5002, /* Fence vertex for an exclusion polygon. The vehicle must stay outside this area. Minimum of 3 vertices required.
         |Polygon vertex count| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
    NAV_RALLY_POINT=5100, /* Rally point. You can have multiple rally points defined.
         |Reserved| Reserved| Reserved| Reserved| Latitude| Longitude| Altitude|  */
    PAYLOAD_PREPARE_DEPLOY=30001, /* Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. |Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.| Desired approach vector in degrees compass heading (0..360). A negative value indicates the system can define the approach vector at will.| Desired ground speed at release time. This can be overriden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.| Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.| Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Altitude, in meters AMSL|  */
    PAYLOAD_CONTROL_DEPLOY=30002, /* Control the payload deployment. |Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deploment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    WAYPOINT_USER_1=31000, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    WAYPOINT_USER_2=31001, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    WAYPOINT_USER_3=31002, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    WAYPOINT_USER_4=31003, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    WAYPOINT_USER_5=31004, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    SPATIAL_USER_1=31005, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    SPATIAL_USER_2=31006, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    SPATIAL_USER_3=31007, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    SPATIAL_USER_4=31008, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    SPATIAL_USER_5=31009, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude, in meters AMSL|  */
    USER_1=31010, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: UXV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    USER_2=31011, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: UXV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    USER_3=31012, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: UXV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    USER_4=31013, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: UXV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    USER_5=31014, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: UXV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
};

//! UXV_CMD ENUM_END
constexpr auto UXV_CMD_ENUM_END = 31015;

/** @brief THIS INTERFACE IS DEPRECATED AS OF JULY 2015. Please use MESSAGE_INTERVAL instead. A data stream is not a fixed set of messages, but rather a
     recommendation to the autopilot software. Individual autopilots may or may not obey
     the recommended messages. */
enum class UXV_DATA_STREAM
{
    ALL=0, /* Enable all data streams | */
    RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets. | */
    EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS | */
    RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW | */
    RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT. | */
    POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages. | */
    EXTRA1=10, /* Dependent on the autopilot | */
    EXTRA2=11, /* Dependent on the autopilot | */
    EXTRA3=12, /* Dependent on the autopilot | */
};

//! UXV_DATA_STREAM ENUM_END
constexpr auto UXV_DATA_STREAM_ENUM_END = 13;

/** @brief ACK / NACK / ERROR values as a result of UXV_CMDs and for mission item transmission. */
enum class UXV_CMD_ACK
{
    OK=1, /* Command / mission item is ok. | */
    ERR_FAIL=2, /* Generic error message if none of the other reasons fails or if no detailed error reporting is implemented. | */
    ERR_ACCESS_DENIED=3, /* The system is refusing to accept this command from this source / communication partner. | */
    ERR_NOT_SUPPORTED=4, /* Command or mission item is not supported, other commands would be accepted. | */
    ERR_COORDINATE_FRAME_NOT_SUPPORTED=5, /* The coordinate frame of this command / mission item is not supported. | */
    ERR_COORDINATES_OUT_OF_RANGE=6, /* The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible. | */
    ERR_X_LAT_OUT_OF_RANGE=7, /* The X or latitude value is out of range. | */
    ERR_Y_LON_OUT_OF_RANGE=8, /* The Y or longitude value is out of range. | */
    ERR_Z_ALT_OUT_OF_RANGE=9, /* The Z or altitude value is out of range. | */
};

//! UXV_CMD_ACK ENUM_END
constexpr auto UXV_CMD_ACK_ENUM_END = 10;

/** @brief Specifies the datatype of a MAVLink parameter. */
enum class UXV_PARAM_TYPE : uint8_t
{
    UINT8=1, /* 8-bit unsigned integer | */
    INT8=2, /* 8-bit signed integer | */
    UINT16=3, /* 16-bit unsigned integer | */
    INT16=4, /* 16-bit signed integer | */
    UINT32=5, /* 32-bit unsigned integer | */
    INT32=6, /* 32-bit signed integer | */
    UINT64=7, /* 64-bit unsigned integer | */
    INT64=8, /* 64-bit signed integer | */
    REAL32=9, /* 32-bit floating-point | */
    REAL64=10, /* 64-bit floating-point | */
};

//! UXV_PARAM_TYPE ENUM_END
constexpr auto UXV_PARAM_TYPE_ENUM_END = 11;

/** @brief result from a mavlink command */
enum class UXV_RESULT : uint8_t
{
    ACCEPTED=0, /* Command ACCEPTED and EXECUTED | */
    TEMPORARILY_REJECTED=1, /* Command TEMPORARY REJECTED/DENIED | */
    DENIED=2, /* Command PERMANENTLY DENIED | */
    UNSUPPORTED=3, /* Command UNKNOWN/UNSUPPORTED | */
    FAILED=4, /* Command executed, but failed | */
};

//! UXV_RESULT ENUM_END
constexpr auto UXV_RESULT_ENUM_END = 5;

/** @brief Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/. */
enum class UXV_SEVERITY : uint8_t
{
    EMERGENCY=0, /* System is unusable. This is a "panic" condition. | */
    ALERT=1, /* Action should be taken immediately. Indicates error in non-critical systems. | */
    CRITICAL=2, /* Action must be taken immediately. Indicates failure in a primary system. | */
    ERROR=3, /* Indicates an error in secondary/redundant systems. | */
    WARNING=4, /* Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. | */
    NOTICE=5, /* An unusual event has occured, though not an error condition. This should be investigated for the root cause. | */
    INFO=6, /* Normal operational messages. Useful for logging. No action is required for these messages. | */
    DEBUG=7, /* Useful non-operational messages that can assist in debugging. These should not occur during normal operation. | */
};

//! UXV_SEVERITY ENUM_END
constexpr auto UXV_SEVERITY_ENUM_END = 8;

/** @brief Power supply status flags (bitmask) */
enum class UXV_POWER_STATUS : uint16_t
{
    BRICK_VALID=1, /* main brick power supply valid | */
    SERVO_VALID=2, /* main servo power supply valid for FMU | */
    USB_CONNECTED=4, /* USB power is connected | */
    PERIPH_OVERCURRENT=8, /* peripheral supply is in over-current state | */
    PERIPH_HIPOWER_OVERCURRENT=16, /* hi-power peripheral supply is in over-current state | */
    CHANGED=32, /* Power status has changed since boot | */
};

//! UXV_POWER_STATUS ENUM_END
constexpr auto UXV_POWER_STATUS_ENUM_END = 33;

/** @brief SERIAL_CONTROL device types */
enum class SERIAL_CONTROL_DEV
{
    TELEM1=0, /* First telemetry port | */
    TELEM2=1, /* Second telemetry port | */
    GPS1=2, /* First GPS port | */
    GPS2=3, /* Second GPS port | */
    SHELL=10, /* system shell | */
};

//! SERIAL_CONTROL_DEV ENUM_END
constexpr auto SERIAL_CONTROL_DEV_ENUM_END = 11;

/** @brief SERIAL_CONTROL flags (bitmask) */
enum class SERIAL_CONTROL_FLAG
{
    REPLY=1, /* Set if this is a reply | */
    RESPOND=2, /* Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message | */
    EXCLUSIVE=4, /* Set if access to the serial port should be removed from whatever driver is currently using it, giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without this flag set | */
    BLOCKING=8, /* Block on writes to the serial port | */
    MULTI=16, /* Send multiple replies until port is drained | */
};

//! SERIAL_CONTROL_FLAG ENUM_END
constexpr auto SERIAL_CONTROL_FLAG_ENUM_END = 17;

/** @brief Enumeration of distance sensor types */
enum class UXV_DISTANCE_SENSOR : uint8_t
{
    LASER=0, /* Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units | */
    ULTRASOUND=1, /* Ultrasound rangefinder, e.g. MaxBotix units | */
    INFRARED=2, /* Infrared rangefinder, e.g. Sharp units | */
};

//! UXV_DISTANCE_SENSOR ENUM_END
constexpr auto UXV_DISTANCE_SENSOR_ENUM_END = 3;

/** @brief Enumeration of sensor orientation, according to its rotations */
enum class UXV_SENSOR_ORIENTATION : uint8_t
{
    ROTATION_NONE=0, /* Roll: 0, Pitch: 0, Yaw: 0 | */
    ROTATION_YAW_45=1, /* Roll: 0, Pitch: 0, Yaw: 45 | */
    ROTATION_YAW_90=2, /* Roll: 0, Pitch: 0, Yaw: 90 | */
    ROTATION_YAW_135=3, /* Roll: 0, Pitch: 0, Yaw: 135 | */
    ROTATION_YAW_180=4, /* Roll: 0, Pitch: 0, Yaw: 180 | */
    ROTATION_YAW_225=5, /* Roll: 0, Pitch: 0, Yaw: 225 | */
    ROTATION_YAW_270=6, /* Roll: 0, Pitch: 0, Yaw: 270 | */
    ROTATION_YAW_315=7, /* Roll: 0, Pitch: 0, Yaw: 315 | */
    ROTATION_ROLL_180=8, /* Roll: 180, Pitch: 0, Yaw: 0 | */
    ROTATION_ROLL_180_YAW_45=9, /* Roll: 180, Pitch: 0, Yaw: 45 | */
    ROTATION_ROLL_180_YAW_90=10, /* Roll: 180, Pitch: 0, Yaw: 90 | */
    ROTATION_ROLL_180_YAW_135=11, /* Roll: 180, Pitch: 0, Yaw: 135 | */
    ROTATION_PITCH_180=12, /* Roll: 0, Pitch: 180, Yaw: 0 | */
    ROTATION_ROLL_180_YAW_225=13, /* Roll: 180, Pitch: 0, Yaw: 225 | */
    ROTATION_ROLL_180_YAW_270=14, /* Roll: 180, Pitch: 0, Yaw: 270 | */
    ROTATION_ROLL_180_YAW_315=15, /* Roll: 180, Pitch: 0, Yaw: 315 | */
    ROTATION_ROLL_90=16, /* Roll: 90, Pitch: 0, Yaw: 0 | */
    ROTATION_ROLL_90_YAW_45=17, /* Roll: 90, Pitch: 0, Yaw: 45 | */
    ROTATION_ROLL_90_YAW_90=18, /* Roll: 90, Pitch: 0, Yaw: 90 | */
    ROTATION_ROLL_90_YAW_135=19, /* Roll: 90, Pitch: 0, Yaw: 135 | */
    ROTATION_ROLL_270=20, /* Roll: 270, Pitch: 0, Yaw: 0 | */
    ROTATION_ROLL_270_YAW_45=21, /* Roll: 270, Pitch: 0, Yaw: 45 | */
    ROTATION_ROLL_270_YAW_90=22, /* Roll: 270, Pitch: 0, Yaw: 90 | */
    ROTATION_ROLL_270_YAW_135=23, /* Roll: 270, Pitch: 0, Yaw: 135 | */
    ROTATION_PITCH_90=24, /* Roll: 0, Pitch: 90, Yaw: 0 | */
    ROTATION_PITCH_270=25, /* Roll: 0, Pitch: 270, Yaw: 0 | */
    ROTATION_PITCH_180_YAW_90=26, /* Roll: 0, Pitch: 180, Yaw: 90 | */
    ROTATION_PITCH_180_YAW_270=27, /* Roll: 0, Pitch: 180, Yaw: 270 | */
    ROTATION_ROLL_90_PITCH_90=28, /* Roll: 90, Pitch: 90, Yaw: 0 | */
    ROTATION_ROLL_180_PITCH_90=29, /* Roll: 180, Pitch: 90, Yaw: 0 | */
    ROTATION_ROLL_270_PITCH_90=30, /* Roll: 270, Pitch: 90, Yaw: 0 | */
    ROTATION_ROLL_90_PITCH_180=31, /* Roll: 90, Pitch: 180, Yaw: 0 | */
    ROTATION_ROLL_270_PITCH_180=32, /* Roll: 270, Pitch: 180, Yaw: 0 | */
    ROTATION_ROLL_90_PITCH_270=33, /* Roll: 90, Pitch: 270, Yaw: 0 | */
    ROTATION_ROLL_180_PITCH_270=34, /* Roll: 180, Pitch: 270, Yaw: 0 | */
    ROTATION_ROLL_270_PITCH_270=35, /* Roll: 270, Pitch: 270, Yaw: 0 | */
    ROTATION_ROLL_90_PITCH_180_YAW_90=36, /* Roll: 90, Pitch: 180, Yaw: 90 | */
    ROTATION_ROLL_90_YAW_270=37, /* Roll: 90, Pitch: 0, Yaw: 270 | */
    ROTATION_ROLL_315_PITCH_315_YAW_315=38, /* Roll: 315, Pitch: 315, Yaw: 315 | */
};

//! UXV_SENSOR_ORIENTATION ENUM_END
constexpr auto UXV_SENSOR_ORIENTATION_ENUM_END = 39;

/** @brief Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability. */
enum class UXV_PROTOCOL_CAPABILITY
{
    MISSION_FLOAT=1, /* Autopilot supports MISSION float message type. | */
    PARAM_FLOAT=2, /* Autopilot supports the new param float message type. | */
    MISSION_INT=4, /* Autopilot supports MISSION_INT scaled integer message type. | */
    COMMAND_INT=8, /* Autopilot supports COMMAND_INT scaled integer message type. | */
    PARAM_UNION=16, /* Autopilot supports the new param union message type. | */
    FTP=32, /* Autopilot supports the new FILE_TRANSFER_PROTOCOL message type. | */
    SET_ATTITUDE_TARGET=64, /* Autopilot supports commanding attitude offboard. | */
    SET_POSITION_TARGET_LOCAL_NED=128, /* Autopilot supports commanding position and velocity targets in local NED frame. | */
    SET_POSITION_TARGET_GLOBAL_INT=256, /* Autopilot supports commanding position and velocity targets in global scaled integers. | */
    TERRAIN=512, /* Autopilot supports terrain protocol / data handling. | */
    SET_ACTUATOR_TARGET=1024, /* Autopilot supports direct actuator control. | */
    FLIGHT_TERMINATION=2048, /* Autopilot supports the flight termination command. | */
    COMPASS_CALIBRATION=4096, /* Autopilot supports onboard compass calibration. | */
    MAVLINK2=8192, /* Autopilot supports mavlink version 2. | */
    MISSION_FENCE=16384, /* Autopilot supports mission fence protocol. | */
    MISSION_RALLY=32768, /* Autopilot supports mission rally point protocol. | */
};

//! UXV_PROTOCOL_CAPABILITY ENUM_END
constexpr auto UXV_PROTOCOL_CAPABILITY_ENUM_END = 32769;

/** @brief Enumeration of estimator types */
enum class UXV_ESTIMATOR_TYPE
{
    NAIVE=1, /* This is a naive estimator without any real covariance feedback. | */
    VISION=2, /* Computer vision based estimate. Might be up to scale. | */
    VIO=3, /* Visual-inertial estimate. | */
    GPS=4, /* Plain GPS estimate. | */
    GPS_INS=5, /* Estimator integrating GPS and inertial sensing. | */
};

//! UXV_ESTIMATOR_TYPE ENUM_END
constexpr auto UXV_ESTIMATOR_TYPE_ENUM_END = 6;

/** @brief Enumeration of battery types */
enum class UXV_BATTERY_TYPE : uint8_t
{
    UNKNOWN=0, /* Not specified. | */
    LIPO=1, /* Lithium polymer battery | */
    LIFE=2, /* Lithium-iron-phosphate battery | */
    LION=3, /* Lithium-ION battery | */
    NIMH=4, /* Nickel metal hydride battery | */
};

//! UXV_BATTERY_TYPE ENUM_END
constexpr auto UXV_BATTERY_TYPE_ENUM_END = 5;

/** @brief Enumeration of battery functions */
enum class UXV_BATTERY_FUNCTION : uint8_t
{
    UNKNOWN=0, /* Battery function is unknown | */
    ALL=1, /* Battery supports all flight systems | */
    PROPULSION=2, /* Battery for the propulsion system | */
    AVIONICS=3, /* Avionics battery | */
    TYPE_PAYLOAD=4, /* Payload battery | */
};

//! UXV_BATTERY_FUNCTION ENUM_END
constexpr auto UXV_BATTERY_FUNCTION_ENUM_END = 5;

/** @brief Enumeration of VTOL states */
enum class UXV_VTOL_STATE : uint8_t
{
    UNDEFINED=0, /* MAV is not configured as VTOL | */
    TRANSITION_TO_FW=1, /* VTOL is in transition from multicopter to fixed-wing | */
    TRANSITION_TO_MC=2, /* VTOL is in transition from fixed-wing to multicopter | */
    MC=3, /* VTOL is in multicopter state | */
    FW=4, /* VTOL is in fixed-wing state | */
};

//! UXV_VTOL_STATE ENUM_END
constexpr auto UXV_VTOL_STATE_ENUM_END = 5;

/** @brief Enumeration of landed detector states */
enum class UXV_LANDED_STATE : uint8_t
{
    UNDEFINED=0, /* MAV landed state is unknown | */
    ON_GROUND=1, /* MAV is landed (on ground) | */
    IN_AIR=2, /* MAV is in air | */
    TAKEOFF=3, /* MAV currently taking off | */
    LANDING=4, /* MAV currently landing | */
};

//! UXV_LANDED_STATE ENUM_END
constexpr auto UXV_LANDED_STATE_ENUM_END = 5;

/** @brief Enumeration of the ADSB altimeter types */
enum class ADSB_ALTITUDE_TYPE : uint8_t
{
    PRESSURE_QNH=0, /* Altitude reported from a Baro source using QNH reference | */
    GEOMETRIC=1, /* Altitude reported from a GNSS source | */
};

//! ADSB_ALTITUDE_TYPE ENUM_END
constexpr auto ADSB_ALTITUDE_TYPE_ENUM_END = 2;

/** @brief ADSB classification for the type of vehicle emitting the transponder signal */
enum class ADSB_EMITTER_TYPE : uint8_t
{
    NO_INFO=0, /*  | */
    LIGHT=1, /*  | */
    SMALL=2, /*  | */
    LARGE=3, /*  | */
    HIGH_VORTEX_LARGE=4, /*  | */
    HEAVY=5, /*  | */
    HIGHLY_MANUV=6, /*  | */
    ROTOCRAFT=7, /*  | */
    UNASSIGNED=8, /*  | */
    GLIDER=9, /*  | */
    LIGHTER_AIR=10, /*  | */
    PARACHUTE=11, /*  | */
    ULTRA_LIGHT=12, /*  | */
    UNASSIGNED2=13, /*  | */
    UAV=14, /*  | */
    SPACE=15, /*  | */
    UNASSGINED3=16, /*  | */
    EMERGENCY_SURFACE=17, /*  | */
    SERVICE_SURFACE=18, /*  | */
    POINT_OBSTACLE=19, /*  | */
};

//! ADSB_EMITTER_TYPE ENUM_END
constexpr auto ADSB_EMITTER_TYPE_ENUM_END = 20;

/** @brief These flags indicate status such as data validity of each data source. Set = data valid */
enum class ADSB_FLAGS : uint16_t
{
    VALID_COORDS=1, /*  | */
    VALID_ALTITUDE=2, /*  | */
    VALID_HEADING=4, /*  | */
    VALID_VELOCITY=8, /*  | */
    VALID_CALLSIGN=16, /*  | */
    VALID_SQUAWK=32, /*  | */
    SIMULATED=64, /*  | */
};

//! ADSB_FLAGS ENUM_END
constexpr auto ADSB_FLAGS_ENUM_END = 65;

/** @brief Bitmask of options for the UXV_CMD_DO_REPOSITION */
enum class UXV_DO_REPOSITION_FLAGS
{
    CHANGE_MODE=1, /* The aircraft should immediately transition into guided. This should not be set for follow me applications | */
};

//! UXV_DO_REPOSITION_FLAGS ENUM_END
constexpr auto UXV_DO_REPOSITION_FLAGS_ENUM_END = 2;

/** @brief Flags in EKF_STATUS message */
enum class ESTIMATOR_STATUS_FLAGS
{
    ATTITUDE=1, /* True if the attitude estimate is good | */
    VELOCITY_HORIZ=2, /* True if the horizontal velocity estimate is good | */
    VELOCITY_VERT=4, /* True if the  vertical velocity estimate is good | */
    POS_HORIZ_REL=8, /* True if the horizontal position (relative) estimate is good | */
    POS_HORIZ_ABS=16, /* True if the horizontal position (absolute) estimate is good | */
    POS_VERT_ABS=32, /* True if the vertical position (absolute) estimate is good | */
    POS_VERT_AGL=64, /* True if the vertical position (above ground) estimate is good | */
    CONST_POS_MODE=128, /* True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow) | */
    PRED_POS_HORIZ_REL=256, /* True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate | */
    PRED_POS_HORIZ_ABS=512, /* True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate | */
    GPS_GLITCH=1024, /* True if the EKF has detected a GPS glitch | */
};

//! ESTIMATOR_STATUS_FLAGS ENUM_END
constexpr auto ESTIMATOR_STATUS_FLAGS_ENUM_END = 1025;

/** @brief  */
enum class MOTOR_TEST_THROTTLE_TYPE
{
    PERCENT=0, /* throttle as a percentage from 0 ~ 100 | */
    PWM=1, /* throttle as an absolute PWM value (normally in range of 1000~2000) | */
    PILOT=2, /* throttle pass-through from pilot's transmitter | */
};

//! MOTOR_TEST_THROTTLE_TYPE ENUM_END
constexpr auto MOTOR_TEST_THROTTLE_TYPE_ENUM_END = 3;

/** @brief  */
enum class GPS_INPUT_IGNORE_FLAGS
{
    FLAG_ALT=1, /* ignore altitude field | */
    FLAG_HDOP=2, /* ignore hdop field | */
    FLAG_VDOP=4, /* ignore vdop field | */
    FLAG_VEL_HORIZ=8, /* ignore horizontal velocity field (vn and ve) | */
    FLAG_VEL_VERT=16, /* ignore vertical velocity field (vd) | */
    FLAG_SPEED_ACCURACY=32, /* ignore speed accuracy field | */
    FLAG_HORIZONTAL_ACCURACY=64, /* ignore horizontal accuracy field | */
    FLAG_VERTICAL_ACCURACY=128, /* ignore vertical accuracy field | */
};

//! GPS_INPUT_IGNORE_FLAGS ENUM_END
constexpr auto GPS_INPUT_IGNORE_FLAGS_ENUM_END = 129;

/** @brief Possible actions an aircraft can take to avoid a collision. */
enum class UXV_COLLISION_ACTION : uint8_t
{
    NONE=0, /* Ignore any potential collisions | */
    REPORT=1, /* Report potential collision | */
    ASCEND_OR_DESCEND=2, /* Ascend or Descend to avoid threat | */
    MOVE_HORIZONTALLY=3, /* Move horizontally to avoid threat | */
    MOVE_PERPENDICULAR=4, /* Aircraft to move perpendicular to the collision's velocity vector | */
    RTL=5, /* Aircraft to fly directly back to its launch point | */
    HOVER=6, /* Aircraft to stop in place | */
};

//! UXV_COLLISION_ACTION ENUM_END
constexpr auto UXV_COLLISION_ACTION_ENUM_END = 7;

/** @brief Aircraft-rated danger from this threat. */
enum class UXV_COLLISION_THREAT_LEVEL : uint8_t
{
    NONE=0, /* Not a threat | */
    LOW=1, /* Craft is mildly concerned about this threat | */
    HIGH=2, /* Craft is panicing, and may take actions to avoid threat | */
};

//! UXV_COLLISION_THREAT_LEVEL ENUM_END
constexpr auto UXV_COLLISION_THREAT_LEVEL_ENUM_END = 3;

/** @brief Source of information about this collision. */
enum class UXV_COLLISION_SRC : uint8_t
{
    ADSB=0, /* ID field references ADSB_VEHICLE packets | */
    MAVLINK_GPS_GLOBAL_INT=1, /* ID field references MAVLink SRC ID | */
};

//! UXV_COLLISION_SRC ENUM_END
constexpr auto UXV_COLLISION_SRC_ENUM_END = 2;

/** @brief Type of GPS fix */
enum class GPS_FIX_TYPE : uint8_t
{
    NO_GPS=0, /* No GPS connected | */
    NO_FIX=1, /* No position information, GPS is connected | */
    TYPE_2D_FIX=2, /* 2D position | */
    TYPE_3D_FIX=3, /* 3D position | */
    DGPS=4, /* DGPS/SBAS aided 3D position | */
    RTK_FLOAT=5, /* RTK float, 3D position | */
    RTK_FIXED=6, /* RTK Fixed, 3D position | */
    STATIC=7, /* Static fixed, typically used for base stations | */
};

//! GPS_FIX_TYPE ENUM_END
constexpr auto GPS_FIX_TYPE_ENUM_END = 8;


} // namespace common
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_heartbeat.hpp"
#include "./mavlink_msg_vehicle_mode.hpp"
#include "./mavlink_msg_vehicle_armed.hpp"
#include "./mavlink_msg_battery_status.hpp"
#include "./mavlink_msg_system_time.hpp"
#include "./mavlink_msg_ping.hpp"
#include "./mavlink_msg_change_operator_control.hpp"
#include "./mavlink_msg_change_operator_control_ack.hpp"
#include "./mavlink_msg_auth_key.hpp"
#include "./mavlink_msg_param_request_read.hpp"
#include "./mavlink_msg_param_request_list.hpp"
#include "./mavlink_msg_param_value.hpp"
#include "./mavlink_msg_param_set.hpp"
#include "./mavlink_msg_gps_raw_int.hpp"
#include "./mavlink_msg_gps_status.hpp"
#include "./mavlink_msg_scaled_pressure.hpp"
#include "./mavlink_msg_attitude.hpp"
#include "./mavlink_msg_attitude_rates.hpp"
#include "./mavlink_msg_attitude_state_full.hpp"
#include "./mavlink_msg_attitude_quaternion.hpp"
#include "./mavlink_msg_local_position_ned.hpp"
#include "./mavlink_msg_local_velocity_ned.hpp"
#include "./mavlink_msg_local_state_full_ned.hpp"
#include "./mavlink_msg_global_position_int.hpp"
#include "./mavlink_msg_global_velocity_int.hpp"
#include "./mavlink_msg_global_position_state_full.hpp"
#include "./mavlink_msg_set_gps_global_origin.hpp"
#include "./mavlink_msg_gps_global_origin.hpp"
#include "./mavlink_msg_vfr_hud.hpp"
#include "./mavlink_msg_command_int.hpp"
#include "./mavlink_msg_command_long.hpp"
#include "./mavlink_msg_command_short.hpp"
#include "./mavlink_msg_command_ack.hpp"
#include "./mavlink_msg_command_system_mode.hpp"
#include "./mavlink_msg_system_mode_ack.hpp"
#include "./mavlink_msg_execute_spatial_action.hpp"
#include "./mavlink_msg_execute_spatial_action_ack.hpp"
#include "./mavlink_msg_radio_status.hpp"
#include "./mavlink_msg_timesync.hpp"
#include "./mavlink_msg_power_status.hpp"
#include "./mavlink_msg_distance_sensor.hpp"
#include "./mavlink_msg_altitude.hpp"
#include "./mavlink_msg_message_interval.hpp"
#include "./mavlink_msg_extended_sys_state.hpp"
#include "./mavlink_msg_adsb_vehicle.hpp"
#include "./mavlink_msg_collision.hpp"
#include "./mavlink_msg_statustext.hpp"
#include "./mavlink_msg_camera_information.hpp"
#include "./mavlink_msg_camera_settings.hpp"
#include "./mavlink_msg_camera_capture_status.hpp"
#include "./mavlink_msg_camera_image_captured.hpp"
#include "./mavlink_msg_flight_information.hpp"

// base include

