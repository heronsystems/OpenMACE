#ifndef MODULE_BOUNDARY_IDENTIFIER_H
#define MODULE_BOUNDARY_IDENTIFIER_H

#include "mace_core/mace_data.h"
#include "mace_core/module_characteristics.h"

//!
//! \brief A wrapper class to wrap a ModuleCharacteristic and boundaryIdentifier
//!
//! This class will be used by external link in two ways:
//! 1. To associate a boundary on own instance with the target being uploaded to.
//! 2. To associate a boundary on a remote instance with a module address to get to it.
//!
class ModuleBoundaryIdentifier
{
public:

    ModuleBoundaryIdentifier();

    ModuleBoundaryIdentifier(const MaceCore::ModuleCharacteristic &module, MaceCore::BoundaryIdentifierType boundaryIdentifier);

    MaceCore::ModuleCharacteristic Module() const;

    MaceCore::BoundaryIdentifierType BoundaryIdentifier() const;

    bool operator==(const ModuleBoundaryIdentifier &rhs) const;


    bool operator!=(const ModuleBoundaryIdentifier &rhs) const;

    bool operator<(const ModuleBoundaryIdentifier &rhs) const;

private:

    std::tuple<MaceCore::ModuleCharacteristic, MaceCore::BoundaryIdentifierType> m_data;
};


namespace std
{
template <>
struct hash<ModuleBoundaryIdentifier>
{
    std::size_t operator()(const ModuleBoundaryIdentifier& k) const
    {
        using std::size_t;
        using std::hash;
        using std::string;

      // Compute individual hash values for first,
      // second and third and combine them using XOR
      // and bit shifting:

        std::size_t const h1 ( std::hash<MaceCore::ModuleCharacteristic>{}(k.Module()) );
        std::size_t const h2 ( std::hash<uint8_t>{}(k.BoundaryIdentifier()) );
        return h1 ^ (h2 << 1);
    }
};
}


#endif // MODULE_BOUNDARY_IDENTIFIER_H
