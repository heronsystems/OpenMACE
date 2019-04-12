#include "boundary_key.h"

namespace BoundaryItem {

BoundaryCharacterisic::BoundaryCharacterisic():
    BoundaryCharacterisic(std::vector<int>(), BOUNDARYTYPE::GENERIC_POLYGON)
{

}

//!
//! \brief Constructor for global boundary
//!
//! A global boundary is true for all vehicles
//! \param boundaryType Type of boundary
//!
BoundaryCharacterisic::BoundaryCharacterisic(const BOUNDARYTYPE &boundaryType) :
    BoundaryCharacterisic(std::vector<int>(), boundaryType)
{

}

//!
//! \brief Constructor for single vehicle
//! \param associatedVehicles Cehicles that is associated with this boundary.
//! \param boundaryType Type of boundary
//!
BoundaryCharacterisic::BoundaryCharacterisic(const int &associatedVehicle, const BOUNDARYTYPE &boundaryType) :
    BoundaryCharacterisic(std::vector<int>({associatedVehicle}), boundaryType)
{

}

BoundaryCharacterisic::BoundaryCharacterisic(const std::vector<int> &associatedVehicles, const BOUNDARYTYPE &boundaryType):
    m_associatedVehicles(associatedVehicles), m_boundaryType(boundaryType)
{
    if(boundaryType == BOUNDARYTYPE::RESOURCE_FENCE)
    {
        if(associatedVehicles.size() != 1)
        {
            throw std::runtime_error("A Resource fence can only be associated with a single vehicle");
        }
    }
}

BoundaryCharacterisic::BoundaryCharacterisic(const BoundaryCharacterisic &obj)
{
    this->m_associatedVehicles = obj.m_associatedVehicles;
    this->m_boundaryType = obj.m_boundaryType;
}



BOUNDARYTYPE BoundaryCharacterisic::Type() const
{
    return m_boundaryType;
}

std::vector<int> BoundaryCharacterisic::List() const
{
    return m_associatedVehicles;
}

//!
//! \brief Determine if the given vehicle is part of this key
//!
//! If the key has no vehicles associated with it, then it is assumed to be global.
//!
//! \param vehicleID Given vehicle ID.
//! \return True if contained
//!
bool BoundaryCharacterisic::ContainsVehicle(const int vehicleID)
{
    if(m_associatedVehicles.size() == 0)
    {
        return true;
    }

    for(auto it = m_associatedVehicles.cbegin() ; it != m_associatedVehicles.cend() ; ++it)
    {
        if(*it == vehicleID)
        {
            return true;
        }
    }

    return false;
}


BoundaryCharacterisic& BoundaryCharacterisic::operator =(const BoundaryCharacterisic &rhs)
{
    this->m_associatedVehicles = rhs.m_associatedVehicles;
    this->m_boundaryType = rhs.m_boundaryType;
    return *this;
}

bool BoundaryCharacterisic::operator <(const BoundaryCharacterisic &rhs) const
{
    if(*this == rhs)
        return false;


    if(this->m_associatedVehicles > rhs.m_associatedVehicles)
        return false;
    if(this->m_boundaryType > rhs.m_boundaryType)
        return false;

    return true;
}

bool BoundaryCharacterisic::operator ==(const BoundaryCharacterisic &rhs) const
{
    if(this->m_associatedVehicles != rhs.m_associatedVehicles)
        return false;
    if(this->m_boundaryType != rhs.m_boundaryType)
        return false;
    return true;
}

bool BoundaryCharacterisic::operator !=(const BoundaryCharacterisic &rhs) const
{
    return !((*this) == rhs);
}

std::ostream& operator<<(std::ostream& os, const BoundaryCharacterisic& t)
{
    std::stringstream stream;
    stream << std::fixed
           << ", Boundary Type" << BoundaryTypeToString(t.m_boundaryType) << ".";
    os << stream.str();

    return os;
}

} //end of namespace BoundaryItem
