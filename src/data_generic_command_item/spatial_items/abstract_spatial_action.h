#ifndef ABSTRACT_SPATIAL_ACTION_H
#define ABSTRACT_SPATIAL_ACTION_H

#include <iostream>
#include <sstream>

#include <mavlink.h>

#include "common/common.h"
#include "common/class_forward.h"

#include "base/pose/cartesian_position_2D.h"
#include "base/pose/cartesian_position_3D.h"

#include "base/pose/geodetic_position_2D.h"
#include "base/pose/geodetic_position_3D.h"

#include "../mission_items/mission_item_interface.h"

#include "../interface_command_helper.h"

#include "../abstract_command_item.h"

#include "data/jsonconverter.h"

namespace command_item {

MACE_CLASS_FORWARD(AbstractSpatialAction);

class AbstractSpatialAction : public AbstractCommandItem, public Interface_CommandHelper<mavlink_command_long_t>, public JSONConverter
{
public:
    AbstractSpatialAction():
        AbstractCommandItem(0,0), position(nullptr)
    {

    }

    AbstractSpatialAction(const unsigned int &systemOrigin, const unsigned int &systemTarget = 0):
        AbstractCommandItem(systemOrigin,systemTarget), position(nullptr)
    {

    }

    virtual ~AbstractSpatialAction() override
    {
        if(position)
        {
            delete position;
            position = nullptr;
        }
    }

    AbstractSpatialAction(const AbstractSpatialAction &copy):
        AbstractCommandItem(copy), position(nullptr)
    {
        //position is a pointer, so we need to deep copy it if it is non-null
        //allocate the memory
        if(copy.position != nullptr)
        {
            //copy the contents
            position = copy.position->getPositionalClone();
        }
    }

    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const override
    {
        QJsonObject json = toJSON_base(vehicleID,dataType);
        getPosition()->updateQJSONObject(json);
        return json;
    }

    virtual void fromJSON(const QJsonDocument &inputJSON)
    {
        mace::pose::GeodeticPosition_3D* tmpObj = position->positionAs<mace::pose::GeodeticPosition_3D>();
        tmpObj->updatePosition(inputJSON.object().value("lat").toDouble(), inputJSON.object().value("lng").toDouble(), inputJSON.object().value("alt").toDouble());
        this->position = tmpObj;
        this->m_name = inputJSON.object().value("name").toString().toStdString();
    }

    virtual std::string toCSV(const std::string &delimiter) const
    {
        const mace::pose::GeodeticPosition_3D* castPosition = position->positionAs<mace::pose::GeodeticPosition_3D>();
        std::string newline = m_name + delimiter + std::to_string(castPosition->getLatitude()) + delimiter + std::to_string(castPosition->getLongitude()) + delimiter + std::to_string(castPosition->getAltitude());
        return newline;
    }
    virtual bool hasSpatialInfluence() const override
    {
        return true;
    }

    bool isPositionSet() const
    {
        return position != nullptr;
    }

    void setPosition(const mace::pose::Position* pos)
    {
        if(position != nullptr){ //first delete and clear the current position
            delete position; position = nullptr;
        }
        position = pos->getPositionalClone();
    }

    mace::pose::Position* getPosition()
    {
        return this->position;
    }

    const mace::pose::Position* getPosition() const
    {
        return this->position;
    }


    /** Interface imposed via AbstractCommandItem */
public: //The logic behind this is that every command item can be used to generate a mission item
    virtual void populateMACECOMMS_MissionItem(mavlink_mace_mission_item_int_t &cmd) const override;

    virtual void fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &obj) override;

    void generateMACEMSG_MissionItem(mavlink_message_t &msg) const override;

    void generateMACEMSG_CommandItem(mavlink_message_t &msg) const override; //we know that you must cast to the specific type to get something explicit based on the command
    /** End of interface imposed via AbstractCommandItem */

    /** Interface imposed via Interface_CommandHelper<mavlink_command_long_t> */
    virtual void populateCommandItem(mavlink_command_long_t &obj) const override;

    virtual void fromCommandItem(const mavlink_command_long_t &obj) override;
    /** End of interface imposed via Interface_CommandHelper<mavlink_command_long_t> */

protected:
    void populatePositionObject(const mace::CoordinateFrameTypes &explicitFrame, const uint8_t &dim, const uint16_t &mask,
                                const double &x=0.0, const double &y=0.0, const double &z=0.0);
public:
    //!
    //! \brief operator =
    //! \param rhs
    //!
    AbstractSpatialAction& operator = (const AbstractSpatialAction &rhs)
    {
        //self-assignment gaurd
        if(this == &rhs)
            return *this;

        AbstractCommandItem::operator =(rhs);

        if(this->position != nullptr){
            delete position; position = nullptr;
        }

        if(rhs.position != nullptr)
        {
            this->position = rhs.position->getPositionalClone();
        }else{ //test if the rhs position is null didnt pass

        }

        return *this;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const AbstractSpatialAction &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }

        if(*this->position != *rhs.position)
        {
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const AbstractSpatialAction &rhs) {
        return !(*this == rhs);
    }

public:

    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    virtual std::string printSpatialCMDInfo() const = 0;

    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printCommandInfo() const override
    {
        return printSpatialCMDInfo();
    }

public:

    //!
    //! \brief position
    //!
    mace::pose::Position* position;

    //!
    //! \brief name
    //!
    std::string m_name;
};

} //end of namespace CommandItem

#endif // ABSTRACT_SPATIAL_ACTION_H
