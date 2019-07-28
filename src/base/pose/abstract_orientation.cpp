#include "abstract_orientation.h"
namespace mace{
namespace pose{

AbstractOrientation::AbstractOrientation(const std::string &name)
{
    this->setObjectName(name);
}
std::string AbstractOrientation::getObjectName() const
{
    return this->name;
}

void AbstractOrientation::setObjectName(const std::string &name)
{
    this->name = name;
}


} //end of namespace pose
} //end of namespace mace
