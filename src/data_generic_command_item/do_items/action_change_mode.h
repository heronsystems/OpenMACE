#ifndef ACTION_CHANGE_MODE_H
#define ACTION_CHANGE_MODE_H

#include <iostream>
#include <string>

#include "common/class_forward.h"

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/interface_command_helper.h"


namespace command_item {

MACE_CLASS_FORWARD(ActionChangeMode);

class ActionChangeMode : public AbstractCommandItem, public Interface_CommandHelper<mavlink_command_int_t>
{
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
    std::shared_ptr<AbstractCommandItem> getClone() const override;

    /**
     * @brief getClone
     * @param state
     */
    void getClone(std::shared_ptr<AbstractCommandItem> &command) const override;

    /** Interface imposed via Interface_CommandItem<mavlink_command_int_t> */
public:
    void populateCommandItem(mavlink_command_int_t &obj) const override;

    void fromCommandItem(const mavlink_command_int_t &obj) override;

    /** End of interface imposed via Interface_CommandItem<mavlink_command_int_t> */


    /** Interface imposed via AbstractCommandItem */
public: //The logic behind this is that every command item can be used to generate a mission item
    void populateMACECOMMS_MissionItem(mavlink_mace_mission_item_int_t &cmd) const override;

    void fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &cmd) override;

    void generateMACEMSG_MissionItem(mavlink_message_t &msg) const override;

    void generateMACEMSG_CommandItem(mavlink_message_t &msg) const override;
/** End of interface imposed via AbstractCommandItem */

public:
    ActionChangeMode();
    ActionChangeMode(const ActionChangeMode &obj);
    ActionChangeMode(const unsigned int &systemOrigin, const unsigned int &systemTarget);

public:
    void setRequestMode(const std::string &mode)
    {
        vehicleMode = mode;
    }

    std::string getRequestMode() const{
        return vehicleMode;
    }


public:
    void operator = (const ActionChangeMode &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->vehicleMode = rhs.vehicleMode;
    }

    bool operator == (const ActionChangeMode &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(this->vehicleMode != rhs.vehicleMode){
            return false;
        }
        return true;
    }

    bool operator != (const ActionChangeMode &rhs) {
        return !(*this == rhs);
    }

public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printCommandInfo() const override;


    friend std::ostream &operator<<(std::ostream &out, const ActionChangeMode &obj)
    {
        out<<"Command Change Mode( Mode: "<<obj.vehicleMode<<")";
        return out;
    }

private:
    std::string vehicleMode;

};

} //end of namespace MissionItem

#endif // ACTION_CHANGE_MODE_H
