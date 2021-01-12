#ifndef TEST_KEY_H
#define TEST_KEY_H

#include <stdint.h>
#include <ostream>
#include "adept_model_types.h"

typedef struct TestKey
{
    public:
    uint16_t testID = 0;    /*!< Locally unique ID assigned by the test creator */
    AdeptModelType blueAgent;
    AdeptModelType redAgent;

    public:
        TestKey &operator=(const TestKey &rhs)
    {
        this->testID = rhs.testID;
        this->blueAgent = rhs.blueAgent;
        this->redAgent = rhs.redAgent;
        return *this;
    }
        std::string toString(){
            return "Test_" + std::to_string(testID) + "_Blue:_" + AdeptModelToString(blueAgent) + "_Red:_" + AdeptModelToString(redAgent);
        }

} TestKey;

inline bool operator==(const TestKey &key1, const TestKey &key2)
{
    return    key1.testID == key2.testID
            && key1.blueAgent == key2.blueAgent
            && key1.redAgent == key2.redAgent;
}

inline bool operator!=(const TestKey &key1, const TestKey &key2)
{
    return !(key1 == key2);
}

// Used for sorting
inline bool operator<(const TestKey &key1, const TestKey &key2)
{
    return key1.testID < key2.testID;
}

inline bool operator>(const TestKey &key1, const TestKey &key2)
{
    return key2 < key1;
}
inline bool operator<=(const TestKey &key1, const TestKey &key2)
{
    return !(key2 < key1);
}
inline bool operator>=(const TestKey &key1, const TestKey &key2)
{
    return !(key1 > key2);
}
#endif // TEST_KEY_H
