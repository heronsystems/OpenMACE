#ifndef ACTION_CHANGE_SPEED_H
#define ACTION_CHANGE_SPEED_H

#include <iostream>

#include "data/speed_frame.h"

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/interface_command_helper.h"


namespace command_item {

class ActionChangeSpeed : public AbstractCommandItem, public Interface_CommandHelper<mavlink_command_long_t>
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
    ActionChangeSpeed();
    ActionChangeSpeed(const ActionChangeSpeed &obj);
    ActionChangeSpeed(const unsigned int &systemOrigin, const unsigned int &systemTarget);

public:
    void setSpeedFrame(const Data::SpeedFrame &frame)
    {
        speedFrame = frame;
    }

    Data::SpeedFrame getSpeedFrame() const
    {
        return speedFrame;
    }

    void setDesiredSpeed(const double &speed)
    {
        desiredSpeed = speed;
    }

    double getDesiredSpeed() const
    {
        return desiredSpeed;
    }

public:
    void operator = (const ActionChangeSpeed &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->speedFrame = rhs.speedFrame;
        this->desiredSpeed = rhs.desiredSpeed;
    }

    bool operator == (const ActionChangeSpeed &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(this->speedFrame != rhs.speedFrame){
            return false;
        }
        if(this->desiredSpeed != rhs.desiredSpeed){
            return false;
        }
        return true;
    }

    bool operator != (const ActionChangeSpeed &rhs) {
        return !(*this == rhs);
    }

public:
    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printCommandInfo() const override;

    friend std::ostream &operator<<(std::ostream &out, const ActionChangeSpeed &obj)
    {
        out<<"Command Change Speed( Frame: "<<Data::SpeedFrameToString(obj.speedFrame)<<", Speed:"<<obj.desiredSpeed<<")";
        return out;
    }

private:
    Data::SpeedFrame speedFrame;
    double desiredSpeed;

};

} //end of namespace CommandItem
#endif // ACTION_CHANGE_SPEED_H
