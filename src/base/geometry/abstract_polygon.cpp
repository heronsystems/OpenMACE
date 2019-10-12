#include "abstract_polygon.h"

namespace mace {
namespace geometry {

Abstract_Polygon::Abstract_Polygon(const std::string &descriptor):
    name(descriptor)
{

}

Abstract_Polygon::Abstract_Polygon(const Abstract_Polygon &copy)
{
    this->name = copy.name;
}


} //end of namespace geometry
} //end of namespace mace

