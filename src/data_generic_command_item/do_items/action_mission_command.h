#ifndef ACTION_MISSION_COMMAND_H
#define ACTION_MISSION_COMMAND_H

#include <iostream>

#include "data_generic_command_item/command_item_type.h"
#include "data/mission_command.h"
#include "data_generic_command_item/abstract_command_item.h"

namespace CommandItem {

class ActionMissionCommand : public AbstractCommandItem
{
public:
    /**
     * @brief getCommandType
     * @return
     */
    COMMANDITEM getCommandType() const override;

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

public:
    ActionMissionCommand();
    ActionMissionCommand(const ActionMissionCommand &obj);
    ActionMissionCommand(const int &systemOrigin, const int &systemTarget);

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
