#ifndef UNITTESTS_POSITION_H
#define UNITTESTS_POSITION_H

#include <map>
#include <string>
#include <vector>
#include <tuple>

#include "common/logging/macelog.h"
#include "../pose/cartesian_position_3D.h"
#include "../pose/geodetic_position_3D.h"
#include "../pose/dynamics_aid.h"

// ********************************* //
// ** Unit test 1 --  ** //
// ********************************* //
inline bool test_PositionTransform_LocalToGlobal(std::string &returnString, std::vector<std::string> &testOutputs) {
    mace::pose::GeodeticPosition_3D originPosition(35.6208989, -78.8033793, 10);
    mace::pose::CartesianPosition_3D initialPosition(10,10,10); //this should amount to the distance being sqrt(200)
    initialPosition.setAltitudeReferenceFrame(mace::AltitudeReferenceTypes::REF_ALT_RELATIVE);

    mace::pose::GeodeticPosition_3D finalPosition;

    mace::pose::GeodeticPosition_3D goalPosition(35.620989, -78.803269, 20);

    mace::pose::DynamicsAid::LocalPositionToGlobal(&originPosition, &initialPosition, &finalPosition);

    double distanceBetween = goalPosition.distanceBetween3D(finalPosition);

    bool success = false;
    returnString = "Unit test 1 -- Position Test Local to Global: FAIL";

    if(distanceBetween < 0.5) {
        success = true;
        returnString = "Unit test 1 -- Position Transformation LocalToGlobal: PASS";
    }

    testOutputs.push_back(returnString);

    return success;
}

// ********************************* //
// ** Unit test 2 --  ** //
// ********************************* //
inline bool test_PositionTransform_GlobalToLocal(std::string &returnString, std::vector<std::string> &testOutputs) {

    mace::pose::GeodeticPosition_3D originPosition(35.6208989, -78.8033793, 10);
    originPosition.setAltitudeReferenceFrame(mace::AltitudeReferenceTypes::REF_ALT_RELATIVE);

    mace::pose::GeodeticPosition_3D finalPosition(35.620989, -78.803269, 20);
    finalPosition.setAltitudeReferenceFrame(mace::AltitudeReferenceTypes::REF_ALT_RELATIVE);


    mace::pose::CartesianPosition_3D targetPosition(10,10,10);
    mace::pose::CartesianPosition_3D localPosition;

    mace::pose::DynamicsAid::GlobalPositionToLocal(&originPosition, &finalPosition, &localPosition);

    double distanceBetween = localPosition.distanceBetween3D(&targetPosition);

    bool success = false;
    returnString = "Unit test 2 -- Position Test Global to Local: FAIL";

    if(distanceBetween < 0.5) {
        success = true;
        returnString = "Unit test 2 -- Position Transformation GlobalToLocal: PASS";
    }

    testOutputs.push_back(returnString);

    return success;
}

// ********************************* //
// ** Unit test 3 --  ** //
// ********************************* //
inline bool test_Position_DistanceBetween(std::string &returnString, std::vector<std::string> &testOutputs) {

    mace::pose::GeodeticPosition_3D firstPosition(35.6208989, -78.8033793, 10);
    mace::pose::GeodeticPosition_3D secondPosition(35.620989, -78.803269, 10);

    double distanceBetween = secondPosition.distanceBetween3D(firstPosition);

    bool success = false;
    returnString = "Unit test 3 -- Position Distance Between Global: FAIL";

    if(fabs(distanceBetween - std::sqrt(200)) < 0.1) {
        success = true;
        returnString = "Unit test 3 -- Position Distance Between Global: PASS";
    }

    testOutputs.push_back(returnString);

    return success;
}

// **************************************** //
// ************* TEST RESULTS ************* //
// **************************************** //
inline void runPositionTests() {
    // Set up variables:
    std::map<std::string, std::tuple<bool, std::string, std::vector<std::string>>> resultsMap;
    std::string rtnStr;
    std::vector<std::string> testOutputs;
    bool success;

    // Unit test 1:
    success = test_PositionTransform_LocalToGlobal(rtnStr, testOutputs);

    std::tuple<bool, std::string, std::vector<std::string>> resultsTuple = std::make_tuple(success, rtnStr, testOutputs);
    resultsMap.insert(std::make_pair("Position_TransformLocalToGlobal", resultsTuple));
    MaceLog::bgWhite("\n\nUnit test 1 outputs: ");
    for(auto str : testOutputs) {
        MaceLog::Info(str);
    }
    testOutputs.clear();

    // Unit test 2:
    success = test_PositionTransform_GlobalToLocal(rtnStr, testOutputs);

    resultsTuple = std::make_tuple(success, rtnStr, testOutputs);
    resultsMap.insert(std::make_pair("Position_TransformGlobalToLocal", resultsTuple));
    MaceLog::bgWhite("\n\nUnit test 2 outputs: ");
    for(auto str : testOutputs) {
        MaceLog::Info(str);
    }
    testOutputs.clear();

    // Unit test 3:
    success = test_Position_DistanceBetween(rtnStr, testOutputs);

    resultsTuple = std::make_tuple(success, rtnStr, testOutputs);
    resultsMap.insert(std::make_pair("Position_DistanceBetween", resultsTuple));
    MaceLog::bgWhite("\n\nUnit test 3 outputs: ");
    for(auto str : testOutputs) {
        MaceLog::Info(str);
    }
    testOutputs.clear();

    // Print pass/fail results
    MaceLog::bgWhite("\n\nTest Results: ");
    for(auto test : resultsMap) {
        if(std::get<0>(test.second)) {
            MaceLog::Green(std::get<1>(test.second));
        }
        else {
            MaceLog::Red(std::get<1>(test.second));
        }
    }

}

#endif // UNITTESTS_POSITION_H
