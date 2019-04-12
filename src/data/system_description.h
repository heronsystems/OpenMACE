#ifndef SYSTEM_DESCRIPTION_H
#define SYSTEM_DESCRIPTION_H

namespace Data
{
class SystemDescription
{
public:
    SystemDescription();
    SystemDescription(const int &systemID);
    SystemDescription(const int &systemID, const int &systemComp);

public:
    int getSystemID() const {
        return systemID;
    }

    void setSystemID(const int &systemID)
    {
        this->systemID = systemID;
    }

    int getSystemComp() const {
        return systemComp;
    }

    void setSystemComp(const int &systemComp)
    {
        this->systemComp = systemComp;
    }


public:
    void operator = (const SystemDescription &rhs)
    {
        this->systemID = rhs.systemID;
        this->systemComp = rhs.systemComp;
    }

    bool operator == (const SystemDescription &rhs) {
        if(this->systemID != rhs.systemID){
            return false;
        }
        if(this->systemComp != rhs.systemComp){
            return false;
        }
        return true;
    }

    bool operator != (const SystemDescription &rhs) {
        return !(*this == rhs);
    }

private:
    int systemID;
    int systemComp;
};
} //end of namespace Data

#endif // SYSTEM_DESCRIPTION_H
