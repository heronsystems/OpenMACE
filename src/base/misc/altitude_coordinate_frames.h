#ifndef ALTITUDE_COORDINATE_FRAMES_H
#define ALTITUDE_COORDINATE_FRAMES_H

#include <string>

namespace mace {

#define ALTITUDE_FRAMES REF_ALT_UNKNOWN, /* Local coordinate frame, Z-up (x: north, y: east, z: down). | */ \
REF_ALT_MSL, /* positive altitude over mean sea level (MSL) | */ \
REF_ALT_RELATIVE, /* positive altitude with 0 being at the altitude of the home location. | */ \
REF_ALT_OFFSET, /* Offset to the current altitude. | */ \
REF_ALT_TERRAIN /* positive altitude in meters with 0 being at ground level in terrain model. | */

enum class AltitudeReferenceTypes: uint8_t{
    ALTITUDE_FRAMES
};

} //end of namespace mace

#endif // ALTITUDE_COORDINATE_FRAMES_H
