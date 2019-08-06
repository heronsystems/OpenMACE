#ifndef BASE_LINE_H
#define BASE_LINE_H

#include <vector>
#include <stdlib.h>
#include <type_traits>

#include "geometry_helper.h"

#include "../misc/kinematic_definitions.h"

#include "../pose/abstract_cartesian_position.h"
#include "../pose/abstract_geodetic_position.h"

namespace mace{
namespace geometry{

template <const CoordinateSystemTypes coordType, class T>
class LineBase
{
    static_assert (std::is_base_of<pose::Position, T>::value,"Line Base template argument T is invalid: Must derive from position."); //This will enforce the interface that we require in order to be able to clone the pointer

public:
    //!
    //! \brief LineBase
    //! \param descriptor
    //!
    LineBase(const std::string &descriptor = "Line"):
        name(descriptor), begin(nullptr), end(nullptr)
    {

    }

    //!
    //! \brief getCoordinateSystemType
    //! \return
    //!
    virtual CoordinateSystemTypes getCoordinateSystemType() const
    {
        return coordType;
    }


    //!
    //! \brief ~LineBase
    //!
    virtual ~LineBase()
    {
        delete begin; begin = nullptr;
        delete end; end = nullptr;
    }

public:
    //!
    //! \brief beginLine
    //! \param obj
    //!
    virtual void beginLine(const T* obj)
    {
        begin = obj->getPositionalClone();
    }

    //!
    //! \brief endLine
    //! \param obj
    //!
    virtual void endLine(const T* obj)
    {
        end = obj->getPositionalClone();
    }

    //!
    //! \brief getBeginLine
    //! \return
    //!
    virtual mace::pose::Position* getBeginLine() const
    {
        return begin;
    }

    //!
    //! \brief getEndLine
    //! \return
    //!
    virtual mace::pose::Position* getEndLine() const
    {
        return end;
    }

protected:
    std::string name;
    mace::pose::Position* begin;
    mace::pose::Position* end;
};

typedef LineBase<CoordinateSystemTypes::CARTESIAN, mace::pose::Abstract_CartesianPosition> Line_Cartesian;
typedef LineBase<CoordinateSystemTypes::GEODETIC, mace::pose::Abstract_GeodeticPosition> Line_Geodetic;

} //end of namepsace geometry
} //end of namespace mace

#endif // BASE_LINE_H
