#ifndef ACTION_MESSAGE_REQUEST_H
#define ACTION_MESSAGE_REQUEST_H

#include <iostream>

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/interface_command_helper.h"

namespace command_item {

class ActionMessageRequest : public AbstractCommandItem, public Interface_CommandHelper<mavlink_command_int_t>
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
    ActionMessageRequest();
    ActionMessageRequest(const ActionMessageRequest &copy);
    ActionMessageRequest(const unsigned int &systemOrigin, const unsigned int &targetSystem);

public:
    void setMessageID(const unsigned int &msgID)
    {
        this->message_id = msgID;
    }

    unsigned int getMessageID() const
    {
        return this->message_id;
    }

public:
    void operator = (const ActionMessageRequest &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->message_id = rhs.message_id;
    }

    bool operator == (const ActionMessageRequest &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(this->message_id != rhs.message_id){
            return false;
        }
        return true;
    }

    bool operator != (const ActionMessageRequest &rhs) {
        return !(*this == rhs);
    }

public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printCommandInfo() const override;

    friend std::ostream &operator<<(std::ostream &out, const ActionMessageRequest &obj)
    {
        UNUSED(obj);
        return out;
    }

private:
    unsigned int message_id;

};

} //end of namespace command_item


#endif // ACTION_MESSAGE_REQUEST_H
