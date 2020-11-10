#ifndef POSE_H
#define POSE_H

#include "data/environment_time.h"
#include "cartesian_position_3D.h"
#include "geodetic_position_3D.h"
#include "rotation_3D.h"

namespace mace{
namespace pose{

MACE_CLASS_FORWARD(Altitude);

class Altitude : public Abstract_Altitude, public Position
{
public:
    Altitude();

    Altitude(const Altitude &copy);

    CoordinateSystemTypes getCoordinateSystemType() const override
    {
        return CoordinateSystemTypes::NOT_IMPLIED;
    }


    CoordinateFrameTypes getExplicitCoordinateFrame() const override
    {
        return CoordinateFrameTypes::CF_NOT_RELEVANT;
    }

    /**
     * @brief getClone
     * @return
     */
    Position* getPositionalClone() const override
    {
        return new Altitude(*this);
    }

    /**
     * @brief getClone
     * @param state
     */
    void getPositionalClone(Position** position) const override
    {
        *position = new Altitude(*this);
    }

    Eigen::VectorXd getDataVector() const override
    {
        Eigen::VectorXd newObject;
        newObject << 0.0, 0.0, z;
        return newObject;
    }

    void updateFromDataVector(const Eigen::VectorXd &vec) override
    {
        size_t rows = vec.rows();
        if(rows == 3)
        {
            this->setAltitude(vec(3));
        }
    }

public:
    //!
    //! \brief setAltitude
    //! \param altitude
    //!
    void setAltitude(const double &altitude) override;

    //!
    //! \brief getAltitude
    //! \return
    //!
    double getAltitude() const override;

    /** Interface imposed via Position */
public:
    PositionTypes getPositionalType() const override
    {
        return PositionTypes::ELEVATION;
    }

    void applyTransformation(const Eigen::Transform<double,2,Eigen::Affine> &t) override;

    void applyTransformation(const Eigen::Transform<double,3,Eigen::Affine> &t) override;
    /** End of interface imposed via Position */
public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printPositionalInfo() const override;

private:
    double z;

};

} //end of namespace pose
} //end of namespace mace
#endif // POSE_H
