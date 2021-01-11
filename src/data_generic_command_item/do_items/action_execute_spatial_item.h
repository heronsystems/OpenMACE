#ifndef ACTION_EXECUTE_SPATIAL_ITEM_H
#define ACTION_EXECUTE_SPATIAL_ITEM_H

#include "common/common.h"
#include "common/class_forward.h"

#include "../spatial_items/spatial_components.h"

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/interface_command_helper.h"


namespace command_item {

MACE_CLASS_FORWARD(Action_ExecuteSpatialItem);

class Action_ExecuteSpatialItem : public AbstractCommandItem
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
    \
    /** Interface imposed via AbstractCommandItem */
public: //The logic behind this is that every command item can be used to generate a mission item
    void populateMACECOMMS_MissionItem(mavlink_mace_mission_item_int_t &cmd) const override;

    void fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &cmd) override;

    void generateMACEMSG_MissionItem(mavlink_message_t &msg) const override;

    void generateMACEMSG_CommandItem(mavlink_message_t &msg) const override;
/** End of interface imposed via AbstractCommandItem */

public:
    Action_ExecuteSpatialItem();
    Action_ExecuteSpatialItem(const AbstractSpatialActionPtr cmd);
    Action_ExecuteSpatialItem(const Action_ExecuteSpatialItem &obj);
    Action_ExecuteSpatialItem(const unsigned int &systemOrigin, const unsigned int &systemTarget);

public:
    void setSpatialAction(const AbstractSpatialActionPtr cmd)
    {
        m_SpatialAction = cmd;
    }

    AbstractSpatialActionPtr getSpatialAction() const{
        return m_SpatialAction;
    }

public:
    void operator = (const Action_ExecuteSpatialItem &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->m_SpatialAction = rhs.m_SpatialAction;
    }

    bool operator == (const Action_ExecuteSpatialItem &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }

        if(this->m_SpatialAction != rhs.m_SpatialAction){
            return false;
        }

        return true;
    }

    bool operator != (const Action_ExecuteSpatialItem &rhs) {
        return !(*this == rhs);
    }
public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printCommandInfo() const override;


    friend std::ostream &operator<<(std::ostream &out, const Action_ExecuteSpatialItem &obj)
    {
        UNUSED(obj);
        //        out<<"Command Change Mode( Mode: "<<obj.vehicleMode<<")";
        return out;
    }

private:
    AbstractSpatialActionPtr m_SpatialAction;

};

} //end of namespace command_item

#endif // ACTION_EXECUTE_SPATIAL_ITEM_H
