#ifndef ABSTRACT_3D_POSITION_H
#define ABSTRACT_3D_POSITION_H

#include <Eigen/Dense>

#include "abstract_position.h"

namespace DataState
{

template <class T>
class Abstract3DPosition : public AbstractPosition<T>
{
public:
    //!
    //! \brief deltaAltitude
    //! \param position
    //! \return
    //!
    virtual double deltaAltitude(const T &position) const = 0;

    //!
    //! \brief distanceBetween3D
    //! \param position
    //! \return
    //!
    virtual double distanceBetween3D(const T &position) const = 0;

public:

    //!
    //! \brief translationTransformation3D
    //! \param position
    //! \param transVec
    //!
    virtual void translationTransformation3D(const T &position, Eigen::Vector3f &transVec) const = 0;

};

} //end of namespace DataState
#endif // ABSTRACT_3D_POSITION_H
