#include "data_generic_item_text.h"

namespace DataGenericItem {

DataGenericItem_Text::DataGenericItem_Text() :
    _severity(MAV_SEVERITY::MAV_SEVERITY_INFO), _dataString("")
{

}

DataGenericItem_Text::DataGenericItem_Text(const DataGenericItem_Text &copyObj)
{
    this->_severity = copyObj.getSeverity();
    this->_dataString = copyObj.getText();
}

DataGenericItem_Text::DataGenericItem_Text(const mavlink_statustext_t &copyObj)
{
    this->_severity = static_cast<MAV_SEVERITY>(copyObj.severity);
    this->_dataString = copyObj.text;
}

mavlink_statustext_t DataGenericItem_Text::getMACECommsObject() const
{
    mavlink_statustext_t rtnObj;

    strcpy(rtnObj.text,this->getText().c_str());
    rtnObj.severity = (uint8_t)this->_severity;

    return rtnObj;
}

mavlink_message_t DataGenericItem_Text::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mavlink_message_t msg;
    mavlink_statustext_t text = getMACECommsObject();
    mavlink_msg_statustext_encode_chan(systemID,compID,chan,&msg,&text);
    return msg;
}

} //end of namespace DataGenericItem
