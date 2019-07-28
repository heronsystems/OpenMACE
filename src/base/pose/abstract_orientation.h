#ifndef ABSTRACT_ORIENTATION_H
#define ABSTRACT_ORIENTATION_H

#include <string>

namespace mace{
namespace pose{

class AbstractOrientation
{
public:
    AbstractOrientation(const std::string &name = "");

    AbstractOrientation(const AbstractOrientation &copy)
    {
        this->name = copy.name;
    }

    virtual ~AbstractOrientation() = default;

    std::string getObjectName() const;

    void setObjectName(const std::string &name);

    AbstractOrientation* getAbstractOrientation() const
    {
        return new AbstractOrientation(*this);
    }

    AbstractOrientation& operator = (const AbstractOrientation &rhs)
    {
        this->name = rhs.name;
        return *this;
    }

public:

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const AbstractOrientation &rhs) const
    {
        if(this->name != rhs.name){
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


    /** Protected Members */
protected:
    std::string name = "";
};



} //end of namespace pose
} //end of namespace mace

#endif // ABSTRACT_ORIENTATION_H
