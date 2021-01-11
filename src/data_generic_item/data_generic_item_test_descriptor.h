#ifndef DATA_GENERIC_ITEM_TEST_DESCRIPTOR_H
#define DATA_GENERIC_ITEM_TEST_DESCRIPTOR_H

#include <iostream>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include "common/common.h"
#include "common/adept_model_types.h"
#include "common/test_key.h"
#include "data/dogfight_teams.h"
#include "data/environment_time.h"
#include "data/jsonconverter.h"

namespace DataGenericItem {

class TestDescriptor: public JSONConverter
{
public:
    TestDescriptor() = default;

    ~TestDescriptor() = default;

public:
    //!
    //! \brief setRoundInfo Populate members from JSON
    //! \param data JSON object containing information about this test
    //!
    void setRoundInfo(const int key, const QJsonDocument &data);

    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const;

    virtual void fromJSON(const QJsonDocument &inputJSON);

    virtual std::string toCSV(const std::string &delimiter) const;

    //!
    //! \brief setKeyModel Assign the appropriate model type in m_key
    //! \param team Team enum for the agent
    //! \param model Model type enum for the agent
    //!
    void setKeyModel(const Data::DogfightTeam &team, const AdeptModelType &model);

    //!///////////////////////////////////////////////
    //! Get Methods
    //!//////////////////////////////////////////////

    TestKey getTestKey() const{
        return m_key;
    }

    std::string getName() const{
        return _testName;
    }

    Data::EnvironmentTime getDateTime() const{
        return _testTime;
    }


    std::string getDescription() const{
        return _testDescription;
    }


    //!///////////////////////////////////////////////
    //! Operators
    //!//////////////////////////////////////////////
public:
    //!
    //! \brief operator = overloaded assignment operator for DataGenericItem_TestDescriptor.
    //! \param rhs object that is to be copied
    //!

    TestDescriptor& operator = (const TestDescriptor &rhs)
    {
        this->m_key = rhs.m_key;
        this->_testName = rhs._testName;
        this->_testTime = rhs._testTime;
        this->_testDescription = rhs._testDescription;
        return *this;
    }

    //!
    //! \brief operator == overloaded comparison operator for DataGenericItem_TestDescriptor.
    //! \param rhs object that the data is compared against.
    //! \return true if the obejcts are equal.
    //!
    bool operator == (const TestDescriptor &rhs) const{
        if(this->m_key != rhs.m_key){
            return false;
        }
        if(this->_testName != rhs._testName){
            return false;
        }
        if(this->_testTime != rhs._testTime){
            return false;
        }
        if(this->_testDescription != rhs._testDescription){
            return false;
        }
        return true;
    }

    //!
    //! \brief operator != overloaded comparison operator for DataGenericItem_TestDescriptor.
    //! \param rhs object that the data is compared against.
    //! \return false if the objects are equal.
    //!
    bool operator != (const TestDescriptor &rhs) const{
        return !(*this == rhs);
    }
protected:
    //!///////////////////////////////////////////////
    //! Member Variables
    //!//////////////////////////////////////////////
    //!
    TestKey m_key;
    std::string _testName;
    Data::EnvironmentTime _testTime;
    std::string _testDescription;
};

} //end of namespace DataGenericItem
#endif // DATA_GENERIC_ITEM_TEST_DESCRIPTOR_H
