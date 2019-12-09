#include "abstract_rotation.h"
namespace mace{
namespace pose{

AbstractRotation::AbstractRotation(const uint8_t &DOF, const std::string &name)
{
    this->m_DOF = DOF;

    this->setObjectName(name);
}

uint8_t AbstractRotation::getDOF() const
{
    return m_DOF;
}

std::string AbstractRotation::getObjectName() const
{
    return this->name;
}

void AbstractRotation::setObjectName(const std::string &name)
{
    this->name = name;
}


} //end of namespace pose
} //end of namespace mace
