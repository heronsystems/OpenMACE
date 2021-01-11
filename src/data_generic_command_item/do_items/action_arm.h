#ifndef ACTION_ARM_H
#define ACTION_ARM_H

#include <iostream>

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/interface_command_helper.h"

namespace command_item {

class ActionArm : public AbstractCommandItem, public Interface_CommandHelper<mavlink_command_long_t>
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


    /** Interface imposed via Interface_CommandItem<mavlink_command_long_t> */
public:
    void populateCommandItem(mavlink_command_long_t &obj) const override;

    void fromCommandItem(const mavlink_command_long_t &obj) override;

    /** End of interface imposed via Interface_CommandItem<mavlink_command_long_t> */


    /** Interface imposed via AbstractCommandItem */
public: //The logic behind this is that every command item can be used to generate a mission item
    void populateMACECOMMS_MissionItem(mavlink_mace_mission_item_int_t &cmd) const override;

    void fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &cmd) override;

    void generateMACEMSG_MissionItem(mavlink_message_t &msg) const override;

    void generateMACEMSG_CommandItem(mavlink_message_t &msg) const override;
/** End of interface imposed via AbstractCommandItem */

public:
    ActionArm();
    ActionArm(const ActionArm &obj);
    ActionArm(const unsigned int &systemOrigin, const unsigned int &targetSystem);

public:
    void setVehicleArm(const bool &arm)
    {
        actionArm = arm;
    }

    bool getRequestArm() const{
        return actionArm;
    }

public:
    void operator = (const ActionArm &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->actionArm = rhs.actionArm;
    }

    bool operator == (const ActionArm &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(this->actionArm != rhs.actionArm){
            return false;
        }
        return true;
    }

    bool operator != (const ActionArm &rhs) {
        return !(*this == rhs);
    }

public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printCommandInfo() const override;

    friend std::ostream &operator<<(std::ostream &out, const ActionArm &obj)
    {
        out<<"Command Arm( Request arm: "<<obj.actionArm<<")";
        return out;
    }

private:
    bool actionArm;

};

} //end of namespace command_item

#endif // ACTION_ARM_H
