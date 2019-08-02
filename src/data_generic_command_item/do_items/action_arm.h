#ifndef ACTION_ARM_H
#define ACTION_ARM_H

#include <iostream>

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/interface_command_item.h"

namespace command_item {

class ActionArm : public AbstractCommandItem, public Interface_CommandItem<COMMANDTYPE::CI_ACT_ARM, mace_command_short_t>
{
public:
    /**
     * @brief getCommandType
     * @return
     */
    COMMANDTYPE getCommandType() const override;

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

    /** Interface imposed via Interface_CommandItem<mace_command_short_t> */
    public:
        void toMACEComms_CommandItem(mace_command_short_t &obj) const override;

    /** End of interface imposed via Interface_CommandItem<mace_command_short_t> */

public:
    ActionArm();
    ActionArm(const ActionArm &obj);
    ActionArm(const int &systemOrigin, const int &targetSystem);

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

} //end of namespace MissionItem

#endif // ACTION_ARM_H
