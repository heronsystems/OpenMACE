#ifndef ABSTRACT_ROTATION_H
#define ABSTRACT_ROTATION_H

#include <Eigen/Geometry>

#include <string>
#include <mavlink.h>

#include "common/class_forward.h"

namespace mace{
namespace pose{

MACE_CLASS_FORWARD(AbstractRotation);
MACE_CLASS_FORWARD(Rotation_2D);
MACE_CLASS_FORWARD(Rotation_3D);

class AbstractRotation
{
public:
    AbstractRotation(const uint8_t &DOF, const std::string &name = "");

    AbstractRotation(const AbstractRotation &copy)
    {
        this->m_DOF = copy.m_DOF;
        this->name = copy.name;
    }

    virtual ~AbstractRotation() = default;

    virtual uint8_t getDOF() const;

    std::string getObjectName() const;

    void setObjectName(const std::string &name);

public:
    virtual mavlink_attitude_quaternion_t getMACEQuaternion() const = 0;

    virtual mavlink_attitude_t getMACEEuler() const = 0;

    virtual mavlink_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const = 0;

public:
    virtual void setQuaternion(const Eigen::Quaterniond &rot) = 0;

    virtual Eigen::Quaterniond getQuaternion() const = 0;


public:
    /**
     *
     */
    template <class T>
    const T *rotationAs() const
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<const T *>(this);
    }

    /**
     *
     */
    template <class T>
    T *rotationAs()
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<T *>(this);
    }

    /**
     * @brief getClone
     * @return
     */
    virtual AbstractRotation* getRotationalClone() const = 0;

    /**
     * @brief getClone
     * @param state
     */
    virtual void getRotationalClone(AbstractRotation** rotation) const = 0;


    AbstractRotation& operator = (const AbstractRotation &rhs)
    {
        this->name = rhs.name;
        this->m_DOF = rhs.m_DOF;
        return *this;
    }

public:

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const AbstractRotation &rhs) const
    {
        if(this->m_DOF != rhs.m_DOF){
            return false;
        }
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
    bool operator != (const AbstractRotation &rhs) const {
        return !(*this == rhs);
    }


    /** Protected Members */
protected:
    std::string name = "";

    uint8_t m_DOF = 0;
};



} //end of namespace pose
} //end of namespace mace

#endif // ABSTRACT_ROTATION_H
