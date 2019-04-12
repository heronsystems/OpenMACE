#include "pair_module_boundary_identifier.h"

ModuleBoundaryIdentifier::ModuleBoundaryIdentifier()
{

}

ModuleBoundaryIdentifier::ModuleBoundaryIdentifier(const MaceCore::ModuleCharacteristic &module, MaceCore::BoundaryIdentifierType boundaryIdentifier) :
    m_data(std::make_tuple(module, boundaryIdentifier))
{

}

MaceCore::ModuleCharacteristic ModuleBoundaryIdentifier::Module() const
{
    return std::get<0>(m_data);
}

MaceCore::BoundaryIdentifierType ModuleBoundaryIdentifier::BoundaryIdentifier() const
{
    return std::get<1>(m_data);
}

bool ModuleBoundaryIdentifier::operator==(const ModuleBoundaryIdentifier &rhs) const
{
    bool value = this->m_data == rhs.m_data;
    return value;
}


bool ModuleBoundaryIdentifier::operator!=(const ModuleBoundaryIdentifier &rhs) const
{
    return !(*this == rhs);
}

bool ModuleBoundaryIdentifier::operator<(const ModuleBoundaryIdentifier &rhs) const
{
    return this->m_data < rhs.m_data;
}
