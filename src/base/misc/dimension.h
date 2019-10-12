#ifndef DIMENSION_H
#define DIMENSION_H

namespace dimension {

//!
//! \brief Interface defining how to interact with a dimension's units
//!
template< typename ENUM_TYPE >
class IDimensionUnits
{
    //!
    //! \brief Return the default unit for this dimension
    //! \return Default unit
    //!
    virtual ENUM_TYPE BaseUnit() const = 0;


    //!
    //! \brief Return the amount of the supplied unit in the dimension's default unit.
    //! \return Ratio of default unit to supplied unit.
    //!
    virtual void TransformToBaseUnit() const = 0;


    //!
    //! \brief Convert a value to the default unit from the unit contained in the object
    //! \param value Value to convert
    //! \return Converted value in unit of this object
    //!
    virtual void ConvertToBase() const = 0;


    //!
    //! \brief Convert a value from the default unit to the unit contained in the object
    //! \param value Value to convert
    //! \return Converted value in default unit of the dimension
    //!
    virtual void ConvertFromBase() const = 0;

};

} //end of namespace dimension

#endif // DIMENSION_H
