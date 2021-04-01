#ifndef ACTION_TIMESYNC_H
#define ACTION_TIMESYNC_H

#include "common/common.h"
#include "common/class_forward.h"
#include <iostream>
#include <string>

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/interface_command_helper.h"

#include "data/jsonconverter.h"

namespace command_item {

class Action_Timesync : public AbstractCommandItem, public JSONConverter
{
public:
    Action_Timesync();
    Action_Timesync(const Action_Timesync &obj);
    Action_Timesync(const unsigned int &systemOrigin, const unsigned int &systemTarget);

public:
    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const override;

    virtual void fromJSON(const QJsonDocument &inputJSON);

    virtual std::string toCSV(const std::string &delimiter) const;

public:


    /**
     * @brief getCommandType
     * @return
     */
    MAV_CMD getCommandType() const override;

    /**
     * @brief getDescription
     * @return
     */
    std::string getDescription() const override;


    /**
     * @brief hasSpatialInfluence
     * @return
     */
    bool hasSpatialInfluence() const override;

    /**
     * @brief getClone
     * @return
     */
    std::shared_ptr<command_item::AbstractCommandItem> getClone() const override;

    /**
     * @brief getClone
     * @param state
     */
    void getClone(std::shared_ptr<command_item::AbstractCommandItem> &command) const override;

    //!///////////////////////////////////////////////
    //! TO/FROM MAVLINK Definitions
    //!//////////////////////////////////////////////
public:
    void populateMACECOMMS_Timesync(mavlink_timesync_t &obj) const;

    void fromMACECOMMS_Timesync(const mavlink_timesync_t &obj);

    /** Interface imposed via AbstractCommandItem */
public: //The logic behind this is that every command item can be used to generate a mission item
    void populateMACECOMMS_MissionItem(mavlink_mace_mission_item_int_t &cmd) const override;

    void fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &cmd) override;

    void generateMACEMSG_MissionItem(mavlink_message_t &msg) const override;

    void generateMACEMSG_CommandItem(mavlink_message_t &msg) const override;
/** End of interface imposed via AbstractCommandItem */

public:
    void operator = (const Action_Timesync &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        _tc1 = rhs._tc1;
        _ts1 = rhs._ts1;
    }

    bool operator == (const Action_Timesync &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(_tc1 != rhs._tc1)
        {
            return false;
        }
        if(_ts1 != rhs._ts1)
        {
            return false;
        }
        return true;
    }

    bool operator != (const Action_Timesync &rhs) {
        return !(*this == rhs);
    }

public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printCommandInfo() const override;

    friend std::ostream &operator<<(std::ostream &out, const Action_Timesync &obj)
    {
        UNUSED(obj);
        //out<<"Command Arm( Request arm: "<<obj.actionArm<<")";
        return out;
    }

private:
    uint64_t _tc1;
    uint64_t _ts1;
};

} //end of namespace command_item

#endif // ACTION_TIMESYNC_H
