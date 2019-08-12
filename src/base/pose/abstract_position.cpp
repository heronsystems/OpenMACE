#include "abstract_position.h"

using namespace mace::pose;

const uint16_t Position::ignoreAllPositions;


//!
//! \brief Position::Position
//! \param posName
//!
Position::Position(const std::string &posName):
    Kinematic_BaseInterface(), name(posName)
{
    this->dimensionMask = ignoreAllPositions;
}

//!
//! \brief Position::Position
//! \param copy
//!
Position::Position(const Position &copy):
    Kinematic_BaseInterface(copy)
{
    this->name = copy.name;
}

//!
//! \brief Position::setName
//! \param stringName
//!
void Position::setName(const std::string &stringName)
{
    this->name = stringName;
}

//!
//! \brief Position::getName
//! \return
//!
std::string Position::getName() const
{
    return this->name;
}
