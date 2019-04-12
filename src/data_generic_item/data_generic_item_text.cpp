#include "data_generic_item_text.h"

namespace DataGenericItem {

DataGenericItem_Text::DataGenericItem_Text() :
    severity(STATUS_SEVERITY::STATUS_INFO), dataString("")
{

}

DataGenericItem_Text::DataGenericItem_Text(const DataGenericItem_Text &copyObj)
{
    this->severity = copyObj.getSeverity();
    this->dataString = copyObj.getText();
}

DataGenericItem_Text::DataGenericItem_Text(const mace_statustext_t &copyObj)
{
    this->severity = static_cast<STATUS_SEVERITY>(copyObj.severity);
    this->dataString = copyObj.text;
}

mace_statustext_t DataGenericItem_Text::getMACECommsObject() const
{
    mace_statustext_t rtnObj;

    strcpy(rtnObj.text,this->getText().c_str());
    rtnObj.severity = (uint8_t)this->severity;

    return rtnObj;
}

mace_message_t DataGenericItem_Text::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_message_t msg;
    mace_statustext_t text = getMACECommsObject();
    mace_msg_statustext_encode_chan(systemID,compID,chan,&msg,&text);
    return msg;
}

} //end of namespace DataGenericItem
