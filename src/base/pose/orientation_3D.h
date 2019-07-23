#ifndef ORIENTATION_3D_H
#define ORIENTATION_3D_H

#include <iostream>
#include <cmath>

#include <Eigen/Geometry>
#include <Eigen_Unsupported/EulerAngles>

#include "abstract_orientation.h"

namespace mace {
namespace pose {

typedef Eigen::EulerSystem<Eigen::EULER_Z, Eigen::EULER_Y, Eigen::EULER_X> AgentRotationSystem;
typedef Eigen::EulerAngles<double, AgentRotationSystem> EulerAngleRotation;

/** A class used to the store the heading anlge (phi) rotation as a functional
 * rotation matrix. This is only to be used when the dimension of the space
 * is 3. The storage of this value is often referred to as a special orthogonal
 * matrix SO(3).
 */

class Orientation_3D
{

public:
    //!
    //! \brief Orientation_2D
    //!
    Orientation_3D(const std::string &name = "Orientation 3D");

    //!
    //! \brief Orientation_3D
    //! \param copy
    //!
    Orientation_3D(const Orientation_3D &copy);


    ~Orientation_3D();

    //!
    //! \brief Orientation_3D
    //! \param angle
    //!
    Orientation_3D(const double &roll, const double &pitch, const double &yaw, const std::string &name = "");

public:

    //!
    //! \brief setRotation
    //! \param rotation
    //!
    void setRotation(const Eigen::Matrix3d &rotation);

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

    Orientation_3D operator * (const Eigen::Matrix3d &that) const
    {
        Orientation_3D newObj(*this);
        newObj.m_QRotation = this->m_QRotation * that;
        return newObj;
    }

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    Orientation_3D operator + (const Orientation_3D &that) const
    {
        Orientation_3D newObj(*this);

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
    Orientation_3D operator - (const Orientation_3D &that) const
    {
        Orientation_3D newObj(*this);

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
    bool operator == (const Orientation_3D &rhs) const
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
    bool operator != (const Orientation_3D &rhs) {
        return !(*this == rhs);
    }

    /** Assignment Operators */
public:

    //!
    //! \brief operator =
    //! \param rhs
    //! \return
    //!
    Orientation_3D& operator = (const Orientation_3D &rhs)
    {
        double rhsRoll, rhsPitch, rhsYaw;
        rhs.getDiscreteEuler(rhsRoll, rhsPitch, rhsYaw);
        this->updateFromEuler(rhsRoll, rhsPitch, rhsYaw);
        return *this;
    }

    Orientation_3D operator *= (const Eigen::Matrix3d &that)
    {
        this->m_QRotation = this->m_QRotation * that;
        return *this;
    }

    //!
    //! \brief operator +=
    //! \param that
    //! \return
    //!
    Orientation_3D& operator += (const Orientation_3D &rhs)
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
    Orientation_3D& operator -= (const Orientation_3D &rhs)
    {
        double currentRoll, currentPitch, currentYaw;
        double rhsRoll, rhsPitch, rhsYaw;

        this->getDiscreteEuler(currentRoll, currentPitch, currentYaw);
        rhs.getDiscreteEuler(rhsRoll, rhsPitch, rhsYaw);

        this->updateFromEuler(currentRoll-rhsRoll, currentPitch-rhsPitch, currentYaw-rhsYaw);
        return *this;
    }

    /** Private Members */
public:
    std::string name = "";

    /** Public Members */
public:

    /** The vector defining the sequence of rotation shall be [yaw, pitch, roll]
     * which yields a [phi, theta, psi] notation.
     * */
    Eigen::Quaterniond m_QRotation;

    /** The vector defining the sequence of rotation shall be [yaw, pitch, roll]
     * which yields a [phi, theta, psi] notation or [Z,Y,X] from conventional literature.
     * */

    /*
    //!
    //! \brief value containing the roll rotation angle
    //!
    mutable double phi = 0.0; //referred to as gamma in literature

    //!
    //! \brief value containing the pitch rotation angle
    //!
    mutable double theta = 0.0; //referred to as beta in literature

    //!
    //! \brief value containing the yaw rotation angle
    //!
    mutable double psi = 0.0; //referred to as alpha in literature
    */

};

} //end of namespace pose
} //end of namespace mace

#endif // ORIENTATION_2D_H
