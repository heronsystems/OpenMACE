#include "system_description.h"
namespace Data
{
SystemDescription::SystemDescription() :
    systemID(0),systemComp(0)
{

}

SystemDescription::SystemDescription(const int &systemID) :
    systemComp(0)
{
    this->systemID = systemID;
}

SystemDescription::SystemDescription(const int &systemID, const int &systemComp)
{
    this->systemID = systemID;
    this->systemComp = systemComp;
}

} //end of namespace Data
