#ifndef MODULE_CHARACTERISTICS_H
#define MODULE_CHARACTERISTICS_H

#include <unordered_set>

namespace MaceCore
{

enum class ModuleClasses
{
    VEHICLE_COMMS = 0,
    EXTERNAL_LINK,
    GROUND_STATION,
    PATH_PLANNING,
    ROS,
    RTA,
    SENSORS,
    NR_TYPES
};


//!
//! \brief Object that "addresses" a module in the global MACE environment
//!
//! A module is addressed by two numbers:
//!  - MaceInstance:    Identifies the MACE instance on the network.
//!  - ModuleID:        Identifing the module on the host MACE instance.
//!
struct ModuleCharacteristic
{
    unsigned int MaceInstance;
    unsigned int ModuleID;

    bool operator== (const ModuleCharacteristic &rhs) const
    {
        if(this->ModuleID != rhs.ModuleID) {
            return false;
        }
        if(this->MaceInstance != rhs.MaceInstance) {
            return false;
        }
        return true;
    }

    bool operator!= (const ModuleCharacteristic &rhs) const
    {
        return !(*this == rhs);
    }

    bool operator < (const ModuleCharacteristic& rhs) const {
        if(this->ModuleID < rhs.ModuleID) {
            return true;
        }
        if(this->MaceInstance < rhs.MaceInstance) {
            return true;
        }
        return false;
    }
};




struct ModuleCharacteristicCmp {
    bool operator()(const ModuleCharacteristic& lhs, const ModuleCharacteristic& rhs) const {

        return lhs < rhs;
    }
};


}


namespace std
{
template <>
struct hash<MaceCore::ModuleCharacteristic>
{
    std::size_t operator()(const MaceCore::ModuleCharacteristic& k) const
    {
        using std::size_t;
        using std::hash;
        using std::string;

      // Compute individual hash values for first,
      // second and third and combine them using XOR
      // and bit shifting:

        std::size_t const h0 ( std::hash<int>{}(k.MaceInstance) );
        std::size_t const h1 ( std::hash<int>{}(k.ModuleID) );
        return h0 ^ (h1 << 1);
    }
};
}

#endif // MODULE_CHARACTERISTICS_H
