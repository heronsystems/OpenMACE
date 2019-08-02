#ifndef ACTION_CHANGE_SPEED_H
#define ACTION_CHANGE_SPEED_H

#include <iostream>

#include "data/speed_frame.h"

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/interface_command_item.h"


namespace command_item {

class ActionChangeSpeed : public AbstractCommandItem, public Interface_CommandItem<COMMANDTYPE::CI_ACT_CHANGESPEED, mace_command_short_t>
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
    ActionChangeSpeed();
    ActionChangeSpeed(const ActionChangeSpeed &obj);
    ActionChangeSpeed(const int &systemOrigin, const int &systemTarget);

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
