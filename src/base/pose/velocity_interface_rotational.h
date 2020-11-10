#ifndef VELOCITY_HELPER_H
#define VELOCITY_HELPER_H

#include <Eigen/Core>
#include <type_traits>

#include "velocity_interface_translational.h"

#include "../abstract_log.h"

namespace pose{

template<const CoordinateSystemTypes coordType, typename CFDATA, class DATA = Eigen::Vector2d>
class VelocityHelper: public VelocityInterface_Translational<coordType, CFDATA, DATA>
{

};

template<const CoordinateSystemTypes coordType, typename CFDATA>
class VelocityHelper<coordType, CFDATA, Eigen::Vector2d> : public VelocityInterface_Translational<coordType, CFDATA, Eigen::Vector2d>, public AbstractLog
{
public:
    VelocityHelper():
        VelocityInterface_Translational<coordType, CFDATA, Eigen::Vector2d>()
    {
        this->dimension = 2;
    }

    VelocityHelper(const CFDATA &frame):
        VelocityInterface_Translational<coordType, CFDATA, Eigen::Vector2d>(frame)
    {
        this->dimension = 2;
    }

    VelocityHelper(const VelocityHelper &copy):
        VelocityInterface_Translational<coordType, CFDATA, Eigen::Vector2d>(copy)
    {

    }

    ~VelocityHelper() = default;

    virtual std::string getLoggingString(const AbstractLog::ACTION_TYPE &action_type = ACTION_TYPE::PROCESS,  const std::string &agentID = "-1", const std::string &message_type = "Velocity_Cartesian2D") const
    {
        std::string line = getLoggingPrefix(action_type, agentID, message_type);
        line += std::to_string(this->getXVelocity()) + m_secondaryDelimeter + std::to_string(this->getYVelocity());
        return line;
    }


public:
    void setXVelocity(const double &value)
    {
        this->data(0) = value;
        this->validateDimension(Velocity::IGNORE_X_DIMENSION);
    }

    void setYVelocity(const double &value)
    {
        this->data(1) = value;
        this->validateDimension(Velocity::IGNORE_Y_DIMENSION);
    }

    double getXVelocity() const
    {
        return this->data(0);
    }

    double getYVelocity() const
    {
        return this->data(1);
    }

    bool hasXBeenSet() const
    {
        if((this->dimensionMask&Velocity::IGNORE_X_DIMENSION) == 0)
            return true;
        return false;
    }

    bool hasYBeenSet() const
    {
        if((this->dimensionMask&Velocity::IGNORE_Y_DIMENSION) == 0)
            return true;
        return false;
    }
};

template<const CoordinateSystemTypes coordType, typename CFDATA>
class VelocityHelper<coordType, CFDATA, Eigen::Vector3d> : public VelocityInterface_Translational<coordType, CFDATA, Eigen::Vector3d>, public AbstractLog
{
public:
    VelocityHelper():
        VelocityInterface_Translational<coordType, CFDATA, Eigen::Vector3d>()
    {
        this->dimension = 3;
    }

    VelocityHelper(const CFDATA &frame):
        VelocityInterface_Translational<coordType, CFDATA, Eigen::Vector3d>(frame)
    {
        this->dimension = 3;
    }

    VelocityHelper(const VelocityHelper &copy):
        VelocityInterface_Translational<coordType, CFDATA, Eigen::Vector3d>(copy)
    {

    }

    ~VelocityHelper() = default;

    virtual std::string getLoggingString(const AbstractLog::ACTION_TYPE &action_type = ACTION_TYPE::PROCESS, const std::string &agentID = "-1", const std::string &message_type = "Velocity_Cartesian3D") const
    {
        std::string line = getLoggingPrefix(action_type, agentID, message_type);
        line += std::to_string(this->getXVelocity()) + m_secondaryDelimeter + std::to_string(this->getYVelocity()) + m_secondaryDelimeter + std::to_string(this->getZVelocity());
        return line;
    }

public:

    void updateVelocityComponents(const double &x, const double &y, const double &z)
    {
        setXVelocity(x);
        setYVelocity(y);
        setZVelocity(z);
    }

    void setXVelocity(const double &value)
    {
        this->data(0) = value;
        this->validateDimension(Velocity::IGNORE_X_DIMENSION);
    }

    void setYVelocity(const double &value)
    {
        this->data(1) = value;
        this->validateDimension(Velocity::IGNORE_Y_DIMENSION);
    }

    void setZVelocity(const double &value)
    {
        this->data(2) = value;
        this->validateDimension(Velocity::IGNORE_Z_DIMENSION);
    }

    double getXVelocity() const
    {
        return this->data(0);
    }

    double getYVelocity() const
    {
        return this->data(1);
    }

    double getZVelocity() const
    {
        return this->data(2);
    }

    bool hasXBeenSet() const
    {
        if((this->dimensionMask&Velocity::IGNORE_X_DIMENSION) == 0)
            return true;
        return false;
    }

    bool hasYBeenSet() const
    {
        if((this->dimensionMask&Velocity::IGNORE_Y_DIMENSION) == 0)
            return true;
        return false;
    }

    bool hasZBeenSet() const
    {
        if((this->dimensionMask&Velocity::IGNORE_Z_DIMENSION) == 0)
            return true;
        return false;
    }

public:
    void populateROSMSG(geometry_msgs::msg::Vector3 &msg) const override
    {
        msg.x = getXVelocity();
        msg.y = getYVelocity();
        msg.z = getZVelocity();
    }

    void fromROSMSG(const geometry_msgs::msg::Vector3 &msg) override
    {
        setXVelocity(msg.x);
        setYVelocity(msg.y);
        setZVelocity(msg.z);
    }

};


typedef VelocityHelper<CoordinateSystemTypes::CARTESIAN, CartesianFrameTypes, Eigen::Vector2d> Velocity_Cartesian2D;
typedef VelocityHelper<CoordinateSystemTypes::CARTESIAN, CartesianFrameTypes, Eigen::Vector3d> Velocity_Cartesian3D;


} // end of namespace pose

#endif // VELOCITY_HELPER
