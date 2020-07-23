#ifndef ABSTRACT_POSITION_H
#define ABSTRACT_POSITION_H
#include <QJsonObject>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sstream>
#include <iostream>
#include <exception>

#include "mace.h"

#include "common/common.h"
#include "common/class_forward.h"
#include "base/math/math_components.h"

#include "../misc/coordinate_frame_components.h"


namespace mace{
namespace pose{

MACE_CLASS_FORWARD(Position);

class Position : public Kinematic_BaseInterface
{
public:
    enum TYPEMASK_POSITION : uint16_t
    {
        POSITION_VALID = 0,
        IGNORE_X_DIMENSION = 1,
        IGNORE_Y_DIMENSION = 2,
        IGNORE_Z_DIMENSION = 4
    };

public:
    static const uint16_t ignoreAllPositions = IGNORE_X_DIMENSION|IGNORE_Y_DIMENSION|IGNORE_Z_DIMENSION;

public:
    //!
    //! \brief Position
    //! \param posName
    //!
    Position(const std::string &posName = "Position Object");

    //!
    //! \brief Position
    //! \param copy
    //!
    Position(const Position &copy);

    virtual ~Position() override = default;

    /** Interface imposed via Kinemnatic_BaseInterace */
    KinematicTypes getKinematicType() const override
    {
        return KinematicTypes::POSITION;
    }
    /** End of interface imposed via Kinemnatic_BaseInterace */

    virtual PositionTypes getPositionalType() const = 0;

    virtual void applyTransformation(const Eigen::Transform<double,2,Eigen::Affine> &t) = 0;

    virtual void applyTransformation(const Eigen::Transform<double,3,Eigen::Affine> &t) = 0;

public:
    virtual Eigen::VectorXd getDataVector() const = 0;

    bool isAnyPositionValid() const
    {
        return (dimensionMask^ignoreAllPositions) > 0 ? true : false;
    }

    virtual bool areAllPositionsValid() const
    {
        if(dimensionMask ==0)
            return true;
        return false;
    }

    virtual bool areTranslationalComponentsValid() const
    {
        bool validTranslation = false;
        if((this->dimensionMask&IGNORE_X_DIMENSION) == 0)
            validTranslation = true;
        else if((this->dimensionMask&IGNORE_Y_DIMENSION) == 0)
            validTranslation = true;
        return validTranslation;
    }

public:
    virtual void updateQJSONObject(QJsonObject &obj) const = 0;

public:
    //!
    //! \brief setName
    //! \param nameString
    //!
    void setName(const std::string &nameString);

    //!
    //! \brief getName
    //! \return
    //!
    std::string getName() const;

    //!
    //! \brief getExplicitCoordinateFrame
    //! \return
    //!
    virtual CoordinateFrameTypes getExplicitCoordinateFrame() const = 0;

    virtual mace_message_t getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const = 0;

public:
    /**
     *
     */
    template <class T>
    const T *positionAs() const
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<const T *>(this);
    }

    /**
     *
     */
    template <class T>
    T *positionAs()
    {
        //ensure that we are attempting to cast it to a type of state
        return static_cast<T *>(this);
    }

    /**
     * @brief getClone
     * @return
     */
    virtual Position* getPositionalClone() const = 0;

    /**
     * @brief getClone
     * @param state
     */
    virtual void getPositionalClone(Position** position) const = 0;


public:
    Position& operator = (const Position &rhs)
    {
        this->name = rhs.name;
        this->dimension = rhs.dimension;
        this->dimensionMask = rhs.dimensionMask;
        return *this;
    }

    bool operator == (const Position &rhs) const
    {
        if(this->dimension != rhs.dimension){
            return false;
        }
        if(this->name != rhs.name){
            return false;
        }
        if(this->dimensionMask != rhs.dimensionMask){
            return false;
        }
        return true;
    }

    bool operator !=(const Position &rhs) const
    {
        return !(*this == rhs);
    }

public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    virtual std::string printPositionalInfo() const = 0;

    //!
    //! \brief printPositionLog
    //! \param os
    //!
    virtual void printPositionLog(std::stringstream &stream) const
    {
        stream << "POS|" <<name<<"|"<<std::to_string(dimension)<<"|"<<CoordinateFrameToString(this->getExplicitCoordinateFrame())<<"|";
        stream << printPositionalInfo();
    }

protected:
    std::string name;
};





} // end of namespace pose
} // end of namespace mace

#endif // ABSTRACT_POSITION_H
