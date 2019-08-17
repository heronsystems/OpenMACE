#ifndef ACTION_DYNAMIC_TARGET_H
#define ACTION_DYNAMIC_TARGET_H

#include "common/common.h"
#include "common/class_forward.h"

#include "../target_items/dynamic_target.h"
#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/interface_command_helper.h"


namespace command_item {

MACE_CLASS_FORWARD(Action_DynamicTarget);

class Action_DynamicTarget : public AbstractCommandItem
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
    \
    /** Interface imposed via AbstractCommandItem */
public: //The logic behind this is that every command item can be used to generate a mission item
    void populateMACECOMMS_MissionItem(mace_mission_item_t &cmd) const override;

    void fromMACECOMMS_MissionItem(const mace_mission_item_t &cmd) override;

    void generateMACEMSG_MissionItem(mace_message_t &msg) const override;

    void generateMACEMSG_CommandItem(mace_message_t &msg) const override; //we know that you must cast to the specific type to get something explicit based on the command

    /** End of interface imposed via Interface_CommandItem<mace_command_short_t> */

public:
    Action_DynamicTarget();
    Action_DynamicTarget(const command_target::DynamicTarget cmd);
    Action_DynamicTarget(const Action_DynamicTarget &obj);
    Action_DynamicTarget(const unsigned int &systemOrigin, const unsigned int &systemTarget);

public:
    void setDynamicTarget(const command_target::DynamicTarget cmd)
    {
        m_Target = cmd;
    }

    command_target::DynamicTarget getDynamicTarget() const{
        return m_Target;
    }

public:
    void operator = (const Action_DynamicTarget &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->m_Target = rhs.m_Target;
    }

    bool operator == (const Action_DynamicTarget &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }

        if(this->m_Target != rhs.m_Target){
            return false;
        }

        return true;
    }

    bool operator != (const Action_DynamicTarget &rhs) {
        return !(*this == rhs);
    }
public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printCommandInfo() const override;


    friend std::ostream &operator<<(std::ostream &out, const Action_DynamicTarget &obj)
    {
        UNUSED(obj);
        //        out<<"Command Change Mode( Mode: "<<obj.vehicleMode<<")";
        return out;
    }

private:
    command_target::DynamicTarget m_Target;

};

} //end of namespace MissionItem

#endif // ACTION_DYNAMIC_TARGET_H
