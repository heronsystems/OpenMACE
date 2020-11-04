#ifndef DATAGENERICITEM_PARAMVALUE_H
#define DATAGENERICITEM_PARAMVALUE_H

#include <iostream>
#include <limits>

#include "mace.h"

#include "data/jsonconverter.h"
#include "common/common.h"

namespace DataGenericItem {

class DataGenericItem_ParamValue
{
public:
    DataGenericItem_ParamValue();

    DataGenericItem_ParamValue(const DataGenericItem_ParamValue &copyObj);

    void setParamValues(const int &index, const std::string &name, const double &value){
        _indexID = index;
        _parameterID = name;
        _value = value;
    }

    double getValue() const{
        return _value;
    }

    std::string getID() const
    {
        return _parameterID;
    }

public:
    void operator = (const DataGenericItem_ParamValue &rhs)
    {
        this->_indexID = rhs._indexID;
        this->_parameterID = rhs._parameterID;
        this->_value = rhs._value;
    }

    bool operator == (const DataGenericItem_ParamValue &rhs) const {
        if(this->_indexID != rhs._indexID){
            return false;
        }
        if(this->_parameterID != rhs._parameterID){
            return false;
        }
        if(fabs(this->_value - rhs._value) > std::numeric_limits<double>::epsilon()){
            return false;
        }
        return true;
    }

    bool operator != (const DataGenericItem_ParamValue &rhs) const {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out)
    {
        out<<"Paramter Value( ParamterID: "<<std::to_string(_indexID)<<", StringID: "<<_parameterID<<", Value %: "<<std::to_string(_value)<<")";
        return out;
    }

protected:
    int _indexID;
    std::string _parameterID;
    double _value;

};

} //end of namespace DataGenericItem

#endif // DATAGENERICITEM_PARAMVALUE_H
