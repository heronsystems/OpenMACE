#ifndef TRACKANGLE_H
#define TRACKANGLE_H

#include <string>
#include <limits>
#include <cmath>
#include "base/pose/pose_components.h"
//#include "base/misc/data_3d.h"

namespace mace {
namespace measurements {

class TrackAngle
{


public:
    TrackAngle();

    TrackAngle(const int &target);

    TrackAngle(const TrackAngle &copy);

    /**
     * @brief getTrackangleClone
     * @return
     */
    TrackAngle* getTrackangleClone() const
    {
        return new TrackAngle(*this);
    }

    /**
     * @brief getTrackangleClone
     * @param angle
     */
    void getTrackangleClone(TrackAngle** angle) const
    {
        *angle = new TrackAngle(*this);
    }

public:

    //!
    //! \brief calculateFromState Calculate the trackangle based on current pose/attitude and target pose, and store it
    //! \param target Global position of the target
    //! \param ownPose Global position of the requesting aircraft
    //! \param ownAttitude Attitude of the requesting aircraft
    //!
    void calculateFromState(const pose::GeodeticPosition_3D &target, const pose::GeodeticPosition_3D &ownPose, const pose::Rotation_3D &ownAttitude);

    //!
    //! \brief setAngle
    //! \param angle
    //!
    void setAngle(const double &angle);

    //!
    //! \brief getAngle
    //! \return
    //!
    double getAngle() const;

    //!
    //! \brief setTarget
    //! \param targetID
    //!
    void setTarget(const int &targetID);

    //!
    //! \brief getTarget
    //! \return
    //!
    int getTarget() const;
public:
    void operator = (const TrackAngle &rhs)
    {
        this->targetID = rhs.targetID;
        this->angle = rhs.angle;
    }

    bool operator == (const TrackAngle &rhs) {
        if(this->targetID != rhs.targetID){
            return false;
        }
        if(fabs(this->angle - rhs.angle) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        return true;
    }

    bool operator != (const TrackAngle &rhs) {
        return !(*this == rhs);
    }

    bool operator > (const TrackAngle &rhs) {
        return this->angle > rhs.angle;
    }

    bool operator < (const TrackAngle &rhs) {
        return this->angle < rhs.angle;
    }

    bool operator >= (const TrackAngle &rhs) {
        return !(*this < rhs);
    }
    bool operator <= (const TrackAngle &rhs) {
        return !(*this > rhs);
    }




private:
    double angle = 0.0;

    int targetID;

};


} //end of namespace measurement
} //end of namespace mace

#endif // TRACKANGLE_H
