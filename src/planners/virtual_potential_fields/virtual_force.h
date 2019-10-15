#ifndef VIRTUAL_FORCE_H
#define VIRTUAL_FORCE_H

#include <cmath>
#include <limits>

class VPF_ResultingForce
{

    enum class ForceType{
        REPULSIVE,
        ATTRACTION
    };

public:
    VPF_ResultingForce(const double &saturation = 0);
    ~VPF_ResultingForce() = default;

    VPF_ResultingForce(const VPF_ResultingForce &copy);

    double getForceX() const;
    void setForceX(const double &value);
    void addForceX(const double &value);

    double getForceY() const;
    void setForceY(const double &value);
    void addForceY(const double &value);

    double getDirection() const;
    bool isSaturated() const;

    void updateMag_Dir()
    {
        this->updateMagnitude();
        this->updateDirection();
    }

private:
    void updateDirection();
    void updateMagnitude();


    /** Arithmetic Operators */
public:

    VPF_ResultingForce& operator += (const VPF_ResultingForce &rhs)
    {
        this->forceX += rhs.forceX;
        this->forceY += rhs.forceY;
        this->updateMag_Dir();
        return *this;
    }

    //!
    //! \brief operator +
    //! \param that
    //! \return
    //!
    VPF_ResultingForce operator + (const VPF_ResultingForce &that) const
    {
        VPF_ResultingForce newPoint(*this);
        newPoint.setForceX(this->forceX + that.forceX);
        newPoint.setForceY(this->forceY + that.forceY);
        return newPoint;
    }

    //!
    //! \brief operator -
    //! \param that
    //! \return
    //!
    VPF_ResultingForce operator - (const VPF_ResultingForce &that) const
    {
        VPF_ResultingForce newPoint(*this);
        newPoint.setForceX(this->forceX - that.forceX);
        newPoint.setForceY(this->forceY - that.forceY);
        return newPoint;
    }

public:
    //!
    //! \brief operator =
    //! \param rhs
    //!
    VPF_ResultingForce& operator = (const VPF_ResultingForce &rhs)
    {
        this->direction = rhs.direction;
        this->forceX = rhs.forceX;
        this->forceY = rhs.forceY;
        this->saturated = rhs.saturated;
        return *this;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const VPF_ResultingForce &rhs)
    {
        if(std::abs(this->forceX - rhs.forceX) > std::numeric_limits<double>::epsilon())
            return false;
        else if(std::abs(this->forceY - rhs.forceY) > std::numeric_limits<double>::epsilon())
            return false;
        else if(std::abs(this->direction - rhs.direction) > std::numeric_limits<double>::epsilon())
            return false;
        else if(this->saturated != rhs.saturated)
            return false;
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const VPF_ResultingForce &rhs) {
        return !(*this == rhs);
    }

private:
    double saturationMagnitude = 0.0;
    bool saturated = false;
    double forceX = 0.0;
    double forceY = 0.0;
    double direction = 0.0;

};

#endif // virtual_FORCE_H
