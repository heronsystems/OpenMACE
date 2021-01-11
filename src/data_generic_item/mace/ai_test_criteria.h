#ifndef AI_TESTCRITERIA_H
#define AI_TESTCRITERIA_H

#include <iostream>
#include <common/common.h>
#include <base/geometry/polygon_2DG.h>

#include "base/ini_support/INIHelper.h"
#include "base/ini_support/INIReader.h"

#include "base/pose/pose_basic_state.h"
#include "base/vehicle/vehicle_state.h"

namespace DataGenericItem {

class AI_TestCriteria
{
public:
    struct criteria
    {
    private:
        double _dx = 0.0;
        double _dy = 0.0;
        double _dz = 0.0;
        double _droll = 0.0;
        double _dpitch = 0.0;
        double _dyaw = 0.0;
        double _dspeed = 0.0;
    public:
        criteria() = default;

        criteria(const double &x, const double &y, const double &z,
                 const double &roll, const double &pitch, const double &yaw, const double &speed)
        {
            _dx = x; _dy = y; _dz = z;
            _droll = roll; _dpitch = pitch; _dyaw = yaw;
            _dspeed = speed;
        }

    public:
        // ** SETTERS **
        void set_dX(const double &dx);
        void set_dY(const double &dy);
        void set_dZ(const double &dz);
        void set_dRoll(const double &droll);
        void set_dPitch(const double &dpitch);
        void set_dYaw(const double &dyaw);
        void set_dSpeed(const double &dspeed);

        // ** GETTERS **
        double get_dX() const;
        double get_dY() const;
        double get_dZ() const;
        double get_dRoll() const;
        double get_dPitch() const;
        double get_dYaw() const;
        double get_dSpeed() const;

        void operator = (const criteria &rhs)
        {
            _dx = rhs._dx; _dy = rhs._dy; _dz = rhs._dz;
            _droll = rhs._droll; _dpitch =rhs._dpitch; _dyaw =rhs._dyaw;
            _dspeed = rhs._dspeed;
        }

        bool operator == (const criteria &rhs) const {
            if(fabs(this->_dx - rhs._dx) > std::numeric_limits<double>::epsilon()){
                return false;
            }
            if(fabs(this->_dy - rhs._dy) > std::numeric_limits<double>::epsilon()){
                return false;
            }
            if(fabs(this->_dz - rhs._dz) > std::numeric_limits<double>::epsilon()){
                return false;
            }
            if(fabs(this->_droll - rhs._droll) > std::numeric_limits<double>::epsilon()){
                return false;
            }
            if(fabs(this->_dpitch - rhs._dpitch) > std::numeric_limits<double>::epsilon()){
                return false;
            }
            if(fabs(this->_dyaw - rhs._dyaw) > std::numeric_limits<double>::epsilon()){
                return false;
            }
            if(fabs(this->_dspeed - rhs._dspeed) > std::numeric_limits<double>::epsilon()){
                return false;
            }
            return true;
        }

        bool operator != (const criteria &rhs) const {
            return !(*this == rhs);
        }
    };

public:
    AI_TestCriteria();

    AI_TestCriteria(const AI_TestCriteria &copy);

    void setMaxTestDuration(const double &time_sec);

    double getMaxTestDuration() const;

    // ** INI Loading **
    void populateTestConditionsFromINI(const std::string &testConditionsFilePath);

public:
    void operator = (const AI_TestCriteria &rhs)
    {
        _startCriteria = rhs._startCriteria;
        _stopCriteria = rhs._stopCriteria;

        _elapsedEndtime = rhs._elapsedEndtime;
    }

    bool operator == (const AI_TestCriteria &rhs) const{
        if(this->_startCriteria != rhs._startCriteria){
            return false;
        }
        if(this->_stopCriteria != rhs._stopCriteria){
            return false;
        }
        if(fabs(this->_elapsedEndtime - rhs._elapsedEndtime) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        return true;
    }

    bool operator != (const AI_TestCriteria &rhs) const {
        return !(*this == rhs);
    }

public:

    friend std::ostream &operator<<(std::ostream &out, const AI_TestCriteria &obj)
    {
        UNUSED(obj);
        return out;
    }

public:
    // Start criteria:
    criteria _startCriteria;

    // Abort criteria:
    criteria _stopCriteria;

private:
    // End criteria:
    double _elapsedEndtime;
};


} //end of namespace DataGenericItem

#endif // AI_TESTCRITERIA_H
