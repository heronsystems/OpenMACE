#include "data_generic_item_param_value.h"

namespace DataGenericItem {

DataGenericItem_ParamValue::DataGenericItem_ParamValue() :
    _indexID(0), _parameterID(""), _value(0.0)
{

}

DataGenericItem_ParamValue::DataGenericItem_ParamValue(const DataGenericItem_ParamValue &copyObj)
{
    this->_indexID = copyObj._indexID;
    this->_parameterID = copyObj._parameterID;
    this->_value = copyObj._value;
}

} //end of namespace DataGenericItem
