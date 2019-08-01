#ifndef BASE_ALTITUDE_H
#define BASE_ALTITUDE_H

#include <Eigen/Core>

#include "abstract_position.h"
#include "abstract_altitude.h"

namespace mace {
namespace pose {

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
        newObject << z;
        return newObject;
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


public:
    mace_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const override;

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

#endif // BASE_ALTITUDE_H
