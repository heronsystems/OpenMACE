#ifndef ACTION_MISSION_COMMAND_H
#define ACTION_MISSION_COMMAND_H

#include <iostream>

#include "common/common.h"

#include "data/mission_command.h"
#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/interface_command_helper.h"

namespace command_item {

class ActionMissionCommand : public AbstractCommandItem, public Interface_CommandHelper<mavlink_command_int_t>
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
    ActionMissionCommand();
    ActionMissionCommand(const ActionMissionCommand &obj);
    ActionMissionCommand(const unsigned int &systemOrigin, const unsigned int &systemTarget);

public:

    void setMissionCommandType(const Data::MissionCommandAction &action)
    {
        this->missionCommand = action;
    }

    void setMissionStart()
    {
        this->missionCommand = Data::MissionCommandAction::MISSIONCA_START;
    }

    void setMissionPause()
    {
        this->missionCommand = Data::MissionCommandAction::MISSIONCA_PAUSE;
    }

    void setMissionResume()
    {
        this->missionCommand = Data::MissionCommandAction::MISSIONCA_RESUME;
    }

    Data::MissionCommandAction getMissionCommandAction() const
    {
        return missionCommand;
    }

public:
    void operator = (const ActionMissionCommand &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->missionCommand = rhs.missionCommand;
    }

    bool operator == (const ActionMissionCommand &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(this->missionCommand != rhs.missionCommand){
            return false;
        }
        return true;
    }

public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printCommandInfo() const override;

    bool operator != (const ActionMissionCommand &rhs) {
        return !(*this == rhs);
    }

private:
    Data::MissionCommandAction missionCommand;
};

} //end of namespace CommandItem

#endif // ACTION_MISSION_COMMAND_H
