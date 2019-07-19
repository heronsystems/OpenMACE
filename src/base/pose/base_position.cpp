#include "base_position.h"

using namespace mace::pose;

Position::Position(const std::string &posName):
    name(posName)
{

}

Position::Position(const Position &copy)
{
    this->name = copy.name;
    this->dimension = copy.dimension;
}

void Position::updatePositionName(const std::string &stringName)
{
    this->name = stringName;
}

std::string Position::getName() const
{
    return this->name;
}

