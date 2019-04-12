#ifndef OCCUPANCY_DEFINITION_H
#define OCCUPANCY_DEFINITION_H

namespace mace {
namespace maps {

enum class OccupiedResult
{
    UNKNOWN,
    OUTSIDE_ENVIRONMENT,
    OCCUPIED,
    NOT_OCCUPIED
};

} //end of namespace maps
} //end of namespace mace

#endif // OCCUPANCY_DEFINITION_H
