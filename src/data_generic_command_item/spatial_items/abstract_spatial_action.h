#ifndef ABSTRACT_SPATIAL_ACTION_H
#define ABSTRACT_SPATIAL_ACTION_H

#include <iostream>
#include <sstream>

#include "common/common.h"
#include "common/class_forward.h"

#include "mace.h"

#include "data_generic_state_item/base_3d_position.h"

#include "data_generic_command_item/abstract_command_item.h"

using namespace DataState;

namespace CommandItem {

MACE_CLASS_FORWARD(AbstractSpatialAction);

//!
//! \brief The AbstractSpatialCommand class
//!
class AbstractSpatialAction : public AbstractCommandItem
{
public:
    AbstractSpatialAction():
        AbstractCommandItem(0,0)
    {
        position = new Base3DPosition();
    }

    AbstractSpatialAction(const int &systemOrigin, const int &systemTarget = 0):
        AbstractCommandItem(systemOrigin,systemTarget)
    {
        position = new Base3DPosition();
    }

    ~AbstractSpatialAction()
    {
        if(position)
        {
            delete position;
            position = NULL;
        }
    }

    AbstractSpatialAction(const AbstractSpatialAction &copy):
        AbstractCommandItem(copy)
    {
        //position is a pointer, so we need to deep copy it if it is non-null
        //allocate the memory
        position = new Base3DPosition();
        if(copy.position)
        {
            //copy the contents
            *position = *copy.position;
        }
    }

    void setPosition(const Base3DPosition &pos)
    {
        if(this->position != NULL)
        {
            delete this->position;
        }
        position = new Base3DPosition (pos);
    }

    Base3DPosition getPosition()
    {
        return *this->position;
    }

    const Base3DPosition getPosition() const
    {
        return *this->position;
    }


public:
    //Ken Fix: Impose this as a pure virtual for interface with any abstract position formulations
    virtual mace_command_goto_t setGoToCommand(mace_command_goto_t &cmd) const
    {
        UNUSED(cmd);
    }

    virtual void updateFromGoToCommand(const mace_command_goto_t &cmd)
    {
        UNUSED(cmd);
    }

public:
    //!
    //! \brief operator =
    //! \param rhs
    //!
    AbstractSpatialAction& operator = (const AbstractSpatialAction &rhs)
    {
        //self-assignment gaurd
        if(this == &rhs)
            return *this;

        AbstractCommandItem::operator =(rhs);

        if(rhs.position)
        {
            *this->position = *rhs.position;
        }else{ //test is null
            position = new Base3DPosition();
        }

        return *this;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const AbstractSpatialAction &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }

        if(*this->position != *rhs.position)
        {
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const AbstractSpatialAction &rhs) {
        return !(*this == rhs);
    }

    friend std::ostream& operator<<(std::ostream& os, const AbstractSpatialAction& t);

public:

    //!
    //! \brief position
    //!
    Base3DPosition *position;
};

} //end of namespace CommandItem

#endif // ABSTRACT_SPATIAL_ACTION_H
