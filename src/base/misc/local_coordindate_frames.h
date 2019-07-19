#ifndef LOCAL_COORDINDATE_FRAMES_H
#define LOCAL_COORDINDATE_FRAMES_H

#include <string>
#include <stdexcept>

namespace mace {

#define LOCAL_FRAMES CF_LOCAL_UNKNOWN, /* Local coordinate frame, Z-up (x: north, y: east, z: down). | */ \
CF_LOCAL_NED, /* Local coordinate frame, Z-up (x: north, y: east, z: down). | */ \
CF_LOCAL_ENU, /* Local coordinate frame, Z-down (x: east, y: north, z: up) | */ \
CF_LOCAL_OFFSET_NED, /* Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position. | */ \
CF_BODY_NED, /* Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right. | */ \
CF_BODY_OFFSET_NED /* Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east. | */


enum class CartesianFrameTypes: uint8_t{
    LOCAL_FRAMES
};


} //end of namespace mace

#endif // LOCAL_COORDINDATE_FRAMES_H
