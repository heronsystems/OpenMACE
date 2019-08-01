#ifndef ABSTRACT_SPATIAL_ACTION_H
#define ABSTRACT_SPATIAL_ACTION_H

#include <iostream>
#include <sstream>

#include "common/common.h"
#include "common/class_forward.h"

#include "mace.h"

#include "base/pose/abstract_position.h"

#include "base/pose/cartesian_position_2D.h"
#include "base/pose/cartesian_position_3D.h"

#include "base/pose/geodetic_position_2D.h"
#include "base/pose/geodetic_position_3D.h"


#include "data_generic_command_item/abstract_command_item.h"

namespace CommandItem {

MACE_CLASS_FORWARD(AbstractSpatialAction);

//!
//! \brief The AbstractSpatialCommand class
//!
class AbstractSpatialAction : public AbstractCommandItem
{
public:
    AbstractSpatialAction():
        AbstractCommandItem(0,0), position(nullptr)
    {

    }

    AbstractSpatialAction(const int &systemOrigin, const int &systemTarget = 0):
        AbstractCommandItem(systemOrigin,systemTarget), position(nullptr)
    {

    }

    ~AbstractSpatialAction()
    {
        if(position)
        {
            delete position;
            position = nullptr;
        }
    }

    AbstractSpatialAction(const AbstractSpatialAction &copy):
        AbstractCommandItem(copy)
    {
        //position is a pointer, so we need to deep copy it if it is non-null
        //allocate the memory
        if(copy.position)
        {
            //copy the contents
            position = copy.position->getPositionalClone();
        }
    }

    bool isPositionSet() const
    {
        return position != nullptr;
    }

    void setPosition(const mace::pose::Position* pos)
    {
        //first delete and clear the current position
        delete position; position = nullptr;

        position = pos->getPositionalClone();
    }

    mace::pose::Position* getPosition()
    {
        return this->position;
    }

    const mace::pose::Position* getPosition() const
    {
        return this->position;
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
        }else{ //test if the rhs position is null didnt pass
            delete position; position = nullptr;
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

public:

    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    virtual std::string printSpatialCMDInfo() const = 0;

    //!
    //! \brief printPositionalInfo
    //! \return
    //!
    std::string printCommandInfo() const override
    {
        return printSpatialCMDInfo();
    }

public:

    //!
    //! \brief position
    //!
    mace::pose::Position* position;
};

} //end of namespace CommandItem

#endif // ABSTRACT_SPATIAL_ACTION_H
