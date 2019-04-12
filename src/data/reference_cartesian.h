#ifndef REFERENCE_CARTESIAN_H
#define REFERENCE_CARTESIAN_H

#include <string>
#include <stdexcept>

namespace Data
{
enum class ReferenceCartesian{
    REF_CART_ENVIRONMENT_NED=1, /* Cartesian point relative to environment frame, Z-up (x: north, y: east, z: down) | */
    REF_CART_ENVIRONMENT_ENU=2, /* Cartesian point relative to environment frame, Z-down (x: east, y: north, z: up) | */
    REF_CART_ENVIRONMENT_VEHICLE=3, /* Cartesian point relative to environment frame. Axis relative to vehicle's attitude | */
    REF_CART_VEHICLE_NED=4, /* Cartesian point relative to vehicle frame, Z-up (x: north, y: east, z: down) | */
    REF_CART_VEHICLE_ENU=5, /* Cartesian point relative to vehicle frame, Z-down (x: east, y: north, z: up) | */
    REF_CART_VEHICLE_VEHICLE=6, /* Cartesian point relative to vehicle frame. Axis relative to vehicle's attitude | */
    REF_CART_ENUM_END=7, /*  | */
};

inline std::string ReferenceCartesianToString(const ReferenceCartesian &frame) {
    switch (frame) {
    case ReferenceCartesian::REF_CART_ENVIRONMENT_NED:
        return "REF_CART_ENVIRONMENT_NED";
    case ReferenceCartesian::REF_CART_ENVIRONMENT_ENU:
        return "REF_CART_ENVIRONMENT_ENU";
    case ReferenceCartesian::REF_CART_ENVIRONMENT_VEHICLE:
        return "REF_CART_ENVIRONMENT_VEHICLE";
    case ReferenceCartesian::REF_CART_VEHICLE_NED:
        return "REF_CART_VEHICLE_NED";
    case ReferenceCartesian::REF_CART_VEHICLE_ENU:
        return "REF_CART_VEHICLE_ENU";
    case ReferenceCartesian::REF_CART_VEHICLE_VEHICLE:
        return "REF_CART_VEHICLE_VEHICLE";
    default:
        throw std::runtime_error("Unknown altitude reference seen");
    }
}

inline ReferenceCartesian ReferenceCartesianFromString(const std::string &str) {
    if(str == "REF_CART_ENVIRONMENT_NED")
        return ReferenceCartesian::REF_CART_ENVIRONMENT_NED;
    if(str == "REF_CART_ENVIRONMENT_ENU")
        return ReferenceCartesian::REF_CART_ENVIRONMENT_ENU;
    if(str == "REF_CART_ENVIRONMENT_VEHICLE")
        return ReferenceCartesian::REF_CART_ENVIRONMENT_VEHICLE;
    if(str == "REF_CART_VEHICLE_NED")
        return ReferenceCartesian::REF_CART_VEHICLE_NED;
    if(str == "REF_CART_VEHICLE_ENU")
        return ReferenceCartesian::REF_CART_VEHICLE_ENU;
    if(str == "REF_CART_VEHICLE_VEHICLE")
        return ReferenceCartesian::REF_CART_VEHICLE_VEHICLE;
    throw std::runtime_error("Unknown altitude reference seen");
}

} //end of namespace Data

#endif // REFERENCE_CARTESIAN_H
