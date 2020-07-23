#include "spatial_home.h"

namespace command_item {

COMMANDTYPE SpatialHome::getCommandType() const
{
    return COMMANDTYPE::CI_NAV_HOME;
}

std::string SpatialHome::getDescription() const
{
    return "This stores the home location for a vehicle";
}

std::shared_ptr<AbstractCommandItem> SpatialHome::getClone() const
{
    return std::make_shared<SpatialHome>(*this);
}

void SpatialHome::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<SpatialHome>(*this);
}

SpatialHome::SpatialHome():
    AbstractSpatialAction(0,0)
{

}

SpatialHome::SpatialHome(const mace::pose::Position* homePosition):
    AbstractSpatialAction(0,0)
{
    this->setPosition(homePosition);
}

SpatialHome::~SpatialHome()
{

}

SpatialHome::SpatialHome(const SpatialHome &obj):
    AbstractSpatialAction(obj)
{
    this->setPosToCurrent = obj.setPosToCurrent;
}

SpatialHome::SpatialHome(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractSpatialAction(systemOrigin,systemTarget)
{

}

//!
//! \brief printPositionalInfo
//! \return
//!
std::string SpatialHome::printSpatialCMDInfo() const
{
    std::stringstream ss;
    if(isPositionSet())
        this->position->printPositionLog(ss);
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const SpatialHome& t)
{

    std::stringstream stream;
    stream.precision(6);
    //stream << std::fixed << "Spatial Home: "<< t.position->getX() << ", "<< t.position->getY() << ", "<< t.position->getZ() << ".";
    os << stream.str();

    return os;
}

} //end of namespace command_item
