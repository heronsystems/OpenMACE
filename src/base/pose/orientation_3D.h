#ifndef ORIENTATION_3D_H
#define ORIENTATION_3D_H

#include <cmath>

#include "Eigen/Core"

#include "abstract_orientation.h"

namespace mace {
namespace pose {

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
    Orientation_3D();

    ~Orientation_3D();

    //!
    //! \brief Orientation_3D
    //! \param copy
    //!
    Orientation_3D(const Orientation_3D &copy);

    //!
    //! \brief Orientation_3D
    //! \param angle
    //!
    Orientation_3D(const double &roll, const double &pitch, const double &yaw);

public:

    //!
    //! \brief setRotation
    //! \param rotation
    //!
    void setRotation(const Eigen::Matrix3d &rotation);


    //!
    //! \brief getRotationMatrix
    //! \return
    //!
    void getRotationMatrix(Eigen::Matrix3d &rotM) const;


    //!
    //! \brief getRotationVector
    //! \param rotV
    //!
    void getRotationVector(Eigen::Vector3d &rotV) const;

    //!
    //! \brief getEuler
    //! \param roll
    //! \param pitch
    //! \param yaw
    //!
    void getEuler(double &roll, double &pitch, double &yaw) const;

    //!
    //! \brief setEuler
    //! \param roll
    //! \param pitch
    //! \param yaw
    //!
    void setEuler(const double &roll, const double &pitch, const double &yaw);

    //!
    //! \brief setRoll
    //! \param angle
    //!
    void setRoll(const double &angle);

    //!
    //! \brief getRoll
    //! \return
    //!
    double getRoll() const;

    //!
    //! \brief setPitch
    //! \param angle
    //!
    void setPitch(const double &angle);

    //!
    //! \brief getPitch
    //! \return
    //!
    double getPitch() const;

    //!
    //! \brief setYaw
    //! \param angle
    //!
    void setYaw(const double &angle);

    //!
    //! \brief getYaw
    //! \return
    //!
    double getYaw() const;

private:
    //!
    //! \brief updateTrigCache
    //!
    inline void updateEuler() const
    {
        if(updatedEuler)
            return;
        psi = std::atan2(matrixRot(2,0),std::hypot(matrixRot(0,0),matrixRot(1,0)));

        //Check for the gimbal lock case based on some numeric that is sufficiently close to where we cannot decipher the condition
        if((std::fabs(matrixRot(2,1)) + std::fabs(matrixRot(2,2))) < 10 * std::numeric_limits<double>::epsilon()){
            theta = 0.0;
            if (psi > 0)
                phi = std::atan2(matrixRot(1,2), matrixRot(0,2));
            else
                phi = std::atan2(-matrixRot(1,2), -matrixRot(0,2));
        }else{
            theta = std::atan2(matrixRot(2,1), matrixRot(2,2));
            phi = std::atan2(matrixRot(1,0), matrixRot(0,0));
        }

        updatedEuler = true;
    }

    //!
    //! \brief updateMatrix
    //!
    inline void updateMatrix() const
    {
        const double cr = std::cos(theta);
        const double sr = std::sin(theta);

        const double cp = std::cos(psi);
        const double sp = std::sin(psi);

        const double cy = std::cos(phi);
        const double sy = std::sin(phi);

        matrixRot(0,0) = cy * cp;
        matrixRot(0,1) = cy * sp * sr - sy * cr;
        matrixRot(0,2) = cy * sp * cr + sy * sr;

        matrixRot(1,0) = sy * cp;
        matrixRot(1,1) = sy * sp * sr + cy * cr;
        matrixRot(1,2) = sy * sp * cr - cy * sr;

        matrixRot(2,0) = -sp;
        matrixRot(2,1) = cp * sr;
        matrixRot(2,2) = cp * cr;
    }


    /** Arithmetic Operators */
public:

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    Orientation_3D operator + (const Orientation_3D &that) const
    {
        double newPhi = this->phi + that.phi;
        double newTheta = this->theta + that.theta;
        double newPsi = this->psi + that.psi;
        Orientation_3D newObj(newPhi, newTheta, newPsi);
        return newObj;
    }

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    Orientation_3D operator - (const Orientation_3D &that) const
    {
        double newPhi = this->phi + that.phi;
        double newTheta = this->theta + that.theta;
        double newPsi = this->psi + that.psi;
        Orientation_3D newObj(newPhi, newTheta, newPsi);
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
        if(this->theta != rhs.theta){
            return false;
        }
        if(this->psi != rhs.psi){
            return false;
        }
        if(this->phi != rhs.phi){
            return false;
        }
        if(this->matrixRot != rhs.matrixRot){
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
        this->phi = rhs.phi;
        this->theta = rhs.theta;
        this->psi = rhs.psi;
        this->matrixRot = rhs.matrixRot;
        return *this;
    }

    //!
    //! \brief operator +=
    //! \param that
    //! \return
    //!
    Orientation_3D& operator += (const Orientation_3D &rhs)
    {
        this->phi += rhs.phi;
        this->theta += rhs.theta;
        this->psi += rhs.psi;
        this->updateMatrix();
        return *this;
    }

    //!
    //! \brief operator -=
    //! \param that
    //! \return
    //!
    Orientation_3D& operator -= (const Orientation_3D &rhs)
    {
        this->phi -= rhs.phi;
        this->theta -= rhs.theta;
        this->psi -= rhs.psi;
        this->updateMatrix();
        return *this;
    }


    /** Private Members */
private:
    //!
    //! \brief updatedTrig
    //!
    mutable bool updatedEuler = false;

    /** Protected Members */
protected:
    mutable Eigen::Matrix3d matrixRot;

    /** The vector defining the sequence of rotation shall be [yaw, pitch, roll]
     * which yields a [phi, theta, psi] notation.
     * */

    //!
    //! \brief value containing the roll rotation angle
    //!
    mutable double theta = 0.0;

    //!
    //! \brief value containing the pitch rotation angle
    //!
    mutable double psi = 0.0;

    //!
    //! \brief value containing the yaw rotation angle
    //!
    mutable double phi = 0.0;

};

} //end of namespace pose
} //end of namespace mace

#endif // ORIENTATION_2D_H
