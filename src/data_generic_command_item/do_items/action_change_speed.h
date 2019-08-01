#ifndef ACTION_CHANGE_SPEED_H
#define ACTION_CHANGE_SPEED_H

#include <iostream>

#include "data/speed_frame.h"

#include "data_generic_command_item/command_item_type.h"
#include "data_generic_command_item/abstract_command_item.h"


namespace CommandItem {

class ActionChangeSpeed : public AbstractCommandItem
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
