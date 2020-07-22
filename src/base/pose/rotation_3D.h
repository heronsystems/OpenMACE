#ifndef ROTATION_3D_H
#define ROTATION_3D_H

#include <iostream>
#include <cmath>

#include <Eigen/Geometry>
#include <Eigen_Unsupported/EulerAngles>

#include "abstract_rotation.h"

namespace mace {
namespace pose {

typedef Eigen::EulerSystem<Eigen::EULER_Z, Eigen::EULER_Y, Eigen::EULER_X> AgentRotationSystem;
typedef Eigen::EulerAngles<double, AgentRotationSystem> EulerAngleRotation;

/** A class used to the store the heading anlge (phi) rotation as a functional
 * rotation matrix. This is only to be used when the dimension of the space
 * is 3. The storage of this value is often referred to as a special orthogonal
 * matrix SO(3).
 */

class Rotation_3D : public AbstractRotation
{
public:
    //!
    //! \brief Orientation_2D
    //!
    Rotation_3D(const std::string &name = "Rotation 3D");

    //!
    //! \brief Orientation_3D
    //! \param copy
    //!
    Rotation_3D(const Rotation_3D &copy);

    //!
    //! \brief Rotation_3D
    //! \param copy
    //!
    Rotation_3D(const Rotation_2D &copy);

    ~Rotation_3D() override;

    //!
    //! \brief Orientation_3D
    //! \param angle
    //!
    Rotation_3D(const double &roll, const double &pitch, const double &yaw, const std::string &name = "");

public:
    AbstractRotation* getRotationalClone() const override
    {
        return (new Rotation_3D(*this));
    }

    void getRotationalClone(AbstractRotation** state) const override
    {
         *state = new Rotation_3D(*this);
    }

public:

    mace_attitude_quaternion_t getMACEQuaternion() const override;

    mace_attitude_t getMACEEuler() const override;

    mace_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const override;

public:

    //!
    //! \brief setRotation
    //! \param rotation
    //!
    void setQuaternion(const Eigen::Quaterniond &rotation) override;

    //!
    //! \brief getQuaternion
    //! \return
    //!
    Eigen::Quaterniond getQuaternion() const override;


    //!
    //! \brief setRotation
    //! \param rotation
    //!
    void setQuaternion(const Eigen::Matrix3d &rotation);

    //!
    //! \brief getEuler
    //! \param roll
    //! \param pitch
    //! \param yaw
    //!
    void getDiscreteEuler(double &roll, double &pitch, double &yaw) const;

    EulerAngleRotation getEulerRotation() const;

    Eigen::Matrix3d getRotationMatrix() const;

    //!
    //! \brief setEuler
    //! \param roll
    //! \param pitch
    //! \param yaw
    //!
    void updateFromEuler(const double &roll, const double &pitch, const double &yaw);

    //!
    //! \brief setRoll
    //! \param angle
    //!
    void updateRoll(const double &angle);

    //!
    //! \brief getRoll
    //! \return
    //!
    double getRoll() const;

    //!
    //! \brief setPitch
    //! \param angle
    //!
    void updatePitch(const double &angle);

    //!
    //! \brief getPitch
    //! \return
    //!
    double getPitch() const;

    //!
    //! \brief setYaw
    //! \param angle
    //!
    void updateYaw(const double &angle);

    //!
    //! \brief getYaw
    //! \return
    //!
    double getYaw() const;


    /** Arithmetic Operators */
public:

    Rotation_3D operator * (const Eigen::Matrix3d &that) const
    {
        Rotation_3D newObj(*this);
        newObj.m_QRotation = this->m_QRotation * that;
        return newObj;
    }

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    Rotation_3D operator + (const Rotation_3D &that) const
    {
        Rotation_3D newObj(*this);

        double currentRoll, currentPitch, currentYaw;
        double rhsRoll, rhsPitch, rhsYaw;

        newObj.getDiscreteEuler(currentRoll, currentPitch, currentYaw);
        that.getDiscreteEuler(rhsRoll, rhsPitch, rhsYaw);

        newObj.updateFromEuler(currentRoll+rhsRoll, currentPitch+rhsPitch, currentYaw+rhsYaw);

        return newObj;
    }

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    Rotation_3D operator - (const Rotation_3D &that) const
    {
        Rotation_3D newObj(*this);

        double currentRoll, currentPitch, currentYaw;
        double rhsRoll, rhsPitch, rhsYaw;

        newObj.getDiscreteEuler(currentRoll, currentPitch, currentYaw);
        that.getDiscreteEuler(rhsRoll, rhsPitch, rhsYaw);

        newObj.updateFromEuler(currentRoll-rhsRoll, currentPitch-rhsPitch, currentYaw-rhsYaw);

        return newObj;
    }

    /** Relational Operators */
public:
    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const Rotation_3D &rhs) const
    {
        if(!this->m_QRotation.isApprox(rhs.m_QRotation,std::numeric_limits<double>::epsilon())){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const Rotation_3D &rhs) const {
        return !(*this == rhs);
    }

    /** Assignment Operators */
public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    Rotation_3D& operator = (const Rotation_3D &rhs)
    {
        AbstractRotation::operator=(rhs);
        double rhsRoll, rhsPitch, rhsYaw;
        rhs.getDiscreteEuler(rhsRoll, rhsPitch, rhsYaw);
        this->updateFromEuler(rhsRoll, rhsPitch, rhsYaw);
        return *this;
    }

    Rotation_3D operator *= (const Eigen::Matrix3d &that)
    {
        this->m_QRotation = this->m_QRotation * that;
        return *this;
    }

    //!
    //! \brief operator +=
    //! \param that
    //! \return
    //!
    Rotation_3D& operator += (const Rotation_3D &rhs)
    {
        double currentRoll, currentPitch, currentYaw;
        double rhsRoll, rhsPitch, rhsYaw;

        this->getDiscreteEuler(currentRoll, currentPitch, currentYaw);
        rhs.getDiscreteEuler(rhsRoll, rhsPitch, rhsYaw);

        this->updateFromEuler(currentRoll+rhsRoll, currentPitch+rhsPitch, currentYaw+rhsYaw);
        return *this;
    }

    //!
    //! \brief operator -=
    //! \param that
    //! \return
    //!
    Rotation_3D& operator -= (const Rotation_3D &rhs)
    {
        double currentRoll, currentPitch, currentYaw;
        double rhsRoll, rhsPitch, rhsYaw;

        this->getDiscreteEuler(currentRoll, currentPitch, currentYaw);
        rhs.getDiscreteEuler(rhsRoll, rhsPitch, rhsYaw);

        this->updateFromEuler(currentRoll-rhsRoll, currentPitch-rhsPitch, currentYaw-rhsYaw);
        return *this;
    }

    /** Public Members */
public:

    /** The vector defining the sequence of rotation shall be [yaw, pitch, roll]
     * which yields a [phi, theta, psi] notation.
     * */
    Eigen::Quaterniond m_QRotation;

    /** The vector defining the sequence of rotation shall be [yaw, pitch, roll]
     * which yields a [phi, theta, psi] notation or [Z,Y,X] from conventional literature.
     * */

public:
    static const uint8_t rotationalDOF = 3;
};

} //end of namespace pose
} //end of namespace mace

#endif // ROTATION_3D_H
