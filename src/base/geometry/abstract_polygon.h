#ifndef ABSTRACT_POLYGON_H
#define ABSTRACT_POLYGON_H

#include <string>
#include "../misc/kinematic_definitions.h"

namespace mace{
namespace geometry{


class Abstract_Polygon
{
public:
    Abstract_Polygon(const std::string &descriptor = "Polygon");

    Abstract_Polygon(const Abstract_Polygon &copy);

    virtual ~Abstract_Polygon() = default;

    virtual CoordinateSystemTypes getVertexCoordinateSystem() const = 0;

public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    Abstract_Polygon& operator = (const Abstract_Polygon &rhs)
    {
        this->name = rhs.name;
        return *this;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Abstract_Polygon &rhs) const
    {
        if(this->name != rhs.name)
        {
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Abstract_Polygon &rhs) const {
        return !(*this == rhs);
    }

protected:
    std::string name;
};

} //end of namespace geometry
} //end of namespace mace

#endif // ABSTRACT_POLYGON_H
