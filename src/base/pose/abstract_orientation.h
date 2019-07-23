#ifndef ABSTRACT_ORIENTATION_H
#define ABSTRACT_ORIENTATION_H

namespace mace{
namespace pose{

template <class DIM>
class AbstractOrientation
{
public:
    virtual ~AbstractOrientation() = default;

    AbstractOrientation()
    {

    }

    AbstractOrientation(const AbstractOrientation &copy)
    {
        this->data = copy.data;
    }

    AbstractOrientation* getAbstractClone() const
    {
        return new AbstractOrientation(*this);
    }

    AbstractOrientation& operator = (const AbstractOrientation &copy)
    {
        this->data = copy.data;
        return *this;
    }

public:

    //!
    //! \brief is3D
    //! \return
    //!
    bool is3D() const
    {
//        if(mace::misc::details::OrientationTypeHelper<DIM>::static_size > 2)
//            return true;
//        return false;
        return false;
    }

    /** Relational Operators */
public:

    //!
    //! \brief operator <
    //! \param rhs
    //! \return
    //!
    bool operator < (const AbstractOrientation &rhs) const
    {
        if(this->data >= rhs.data)
            return false;
        return true;
    }

    //!
    //! \brief operator >=
    //! \param rhs
    //! \return
    //!
    bool operator >= (const AbstractOrientation &rhs) const
    {
        return !(*this < rhs);
    }

    //!
    //! \brief operator >
    //! \param rhs
    //! \return
    //!
    bool operator > (const AbstractOrientation &rhs) const
    {
        if(this->data <= rhs.data)
            return false;
        return true;
    }

    //!
    //! \brief operator <=
    //! \param rhs
    //! \return
    //!
    bool operator <= (const AbstractOrientation &rhs) const
    {
        return !(*this > rhs);
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const AbstractOrientation &rhs) const
    {
        if(this->data != rhs.data){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const AbstractOrientation &rhs) const {
        return !(*this == rhs);
    }

    /** Assignment Operators */
public:

    //!
    //! \brief operator +=
    //! \param that
    //! \return
    //!
    AbstractOrientation& operator += (const AbstractOrientation &rhs)
    {
        this->data += rhs.data;
        return *this;
    }

    //!
    //! \brief operator -=
    //! \param that
    //! \return
    //!
    AbstractOrientation& operator -= (const AbstractOrientation &rhs)
    {
        this->data -= rhs.data;
        return *this;
    }

protected:
    DIM data;
};



} //end of namespace pose
} //end of namespace mace

#endif // ABSTRACT_ORIENTATION_H
