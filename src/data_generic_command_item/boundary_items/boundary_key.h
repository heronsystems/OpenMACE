#ifndef BOUNDARY_KEY_H
#define BOUNDARY_KEY_H

#include <stdint.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>

#include "boundary_type.h"

#include "mace_core/module_characteristics.h"

namespace BoundaryItem {

class BoundaryCharacterisic
{
private:
    std::vector<int> m_associatedVehicles;

    BOUNDARYTYPE m_boundaryType;

public:
    BoundaryCharacterisic();

    //!
    //! \brief Constructor for global boundary
    //!
    //! A global boundary is true for all vehicles
    //! \param boundaryType Type of boundary
    //!
    BoundaryCharacterisic(const BOUNDARYTYPE &boundaryType);

    //!
    //! \brief Constructor for single vehicle
    //! \param associatedVehicles Cehicles that is associated with this boundary.
    //! \param boundaryType Type of boundary
    //!
    BoundaryCharacterisic(const int &associatedVehicles, const BOUNDARYTYPE &boundaryType);

    //!
    //! \brief Constructor
    //! \param associatedVehicles Vector of vehicles that are associated with this boundary. If zero then boundary is global
    //! \param boundaryType Type of boundary
    //!
    BoundaryCharacterisic(const std::vector<int> &associatedVehicles, const BOUNDARYTYPE &boundaryType);

    BoundaryCharacterisic(const BoundaryCharacterisic &obj);



    BOUNDARYTYPE Type() const;

    std::vector<int> List() const;


    //!
    //! \brief Determine if the given vehicle is part of this key
    //!
    //! If the key has no vehicles associated with it, then it is assumed to be global.
    //!
    //! \param vehicleID Given vehicle ID.
    //! \return True if contained
    //!
    bool ContainsVehicle(const int vehicleID);



    BoundaryCharacterisic& operator =(const BoundaryCharacterisic &rhs);

    bool operator< (const BoundaryCharacterisic &rhs) const;

    bool operator== (const BoundaryCharacterisic &rhs) const;

    bool operator!= (const BoundaryCharacterisic &rhs) const;

    friend std::ostream& operator<<(std::ostream& os, const BoundaryCharacterisic& t);

    std::size_t hash() const
    {
        using std::size_t;
        using std::hash;
        using std::string;

        int vectorHash = 0;
        for(auto it = m_associatedVehicles.cbegin() ; it != m_associatedVehicles.cend() ; ++it)
        {
            vectorHash ^= hash<int>()(*it);
        }

        return (vectorHash
                 ^ (hash<int>()((int)m_boundaryType)));
    }
};

} //end of namespace Data

namespace std
{
template <>
struct hash<BoundaryItem::BoundaryCharacterisic>
{
    std::size_t operator()(const BoundaryItem::BoundaryCharacterisic& k) const
    {
        return k.hash();
    }
};
}
#endif // BOUNDARY_KEY_H
