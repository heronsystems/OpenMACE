#ifndef UNITTESTS_ORIENTATION_H
#define UNITTESTS_ORIENTATION_H

#include <map>
#include <string>
#include <vector>
#include <tuple>

#include "common/logging/macelog.h"
#include "../pose/rotation_2D.h"
#include "../pose/rotation_3D.h"

// ********************************* //
// ** Unit test 1 -- Check Simple Orientation Construction From Euler ** //
// ********************************* //
inline bool test_OrientationConstruction(std::string &returnString, std::vector<std::string> &testOutputs) {

    mace::pose::Rotation_3D yaw45(0,0,M_PI/4);

    Eigen::AngleAxisd comparisonRotation (M_PI/4, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d comparisonMatrix = comparisonRotation.toRotationMatrix();

    bool success = false;
    returnString = "Unit test 1 -- Orientation Construction: FAIL";
    Eigen::Matrix3d yaw45Matrix = yaw45.getRotationMatrix();

    if(yaw45Matrix.isApprox(comparisonMatrix, std::numeric_limits<double>::epsilon())) {
        success = true;
        returnString = "Unit test 1 -- Orientation Construction: PASS";
    }
    return success;
}

// ********************************* //
// ** Unit test 2 -- Check quaternion maps back to Euler ** //
// ********************************* //
inline bool test_EulerReconstruction(std::string &returnString, std::vector<std::string> &testOutputs) {

    mace::pose::Rotation_3D orientationTest(M_PI_4,M_PI_4,M_PI_2);
    double currentRoll = M_PI_4, currentPitch = M_PI_4, currentYaw = M_PI_2;
    double rxRoll, rxPitch, rxYaw;
    orientationTest.getDiscreteEuler(rxRoll, rxPitch, rxYaw);
    double qw, qx, qy, qz;
    qw = orientationTest.m_QRotation.w();
    qx = orientationTest.m_QRotation.x();
    qy = orientationTest.m_QRotation.y();
    qz = orientationTest.m_QRotation.z();
    bool success = false;

    testOutputs.push_back("The delta in the roll is:" + std::to_string(fabs(currentRoll - rxRoll)));
    testOutputs.push_back("The delta in the pitch is:" + std::to_string(fabs(currentPitch - rxPitch)));
    testOutputs.push_back("The delta in the yaw is:" + std::to_string(fabs(currentYaw - rxYaw)));

    returnString = "Unit test 2 -- Euler Reconstruction: FAIL";

    if((fabs(currentRoll - rxRoll) <= 0.0001) &&
            (fabs(currentPitch - rxPitch) <= 0.0001) &&
            (fabs(currentYaw - rxYaw) <= 0.0001))
    {
        success = true;
        returnString = "Unit test 2 -- Euler Reconstruction: PASS";
    }
    return success;
}

// ********************************* //
// ** Unit test 3 -- Check arithmetic for the orientation library ** //
// ********************************* //
inline bool test_EulerArithmetic(std::string &returnString, std::vector<std::string> &testOutputs) {

    mace::pose::Rotation_3D lhs(0,0,M_PI_4);
    mace::pose::Rotation_3D rhs(0,0,M_PI_4);
    mace::pose::Rotation_3D desiredResult(0,0,M_PI_2);
    lhs+=rhs;

    bool success = false;
    returnString = "Unit test 3 -- Euler Arithmetic: FAIL";

    if(lhs == desiredResult)
    {
        success = true;
        returnString = "Unit test 3 -- Euler Arithmetic: PASS";
    }
    return success;
}

// **************************************** //
// ************* TEST RESULTS ************* //
// **************************************** //
inline void runOrientationTests() {
    // Set up variables:
    std::map<std::string, std::tuple<bool, std::string, std::vector<std::string>>> resultsMap;
    std::string rtnStr;
    std::vector<std::string> testOutputs;
    bool success;

    // Unit test 1:
    success = test_OrientationConstruction(rtnStr, testOutputs);

    std::tuple<bool, std::string, std::vector<std::string>> resultsTuple = std::make_tuple(success, rtnStr, testOutputs);
    resultsMap.insert(std::make_pair("OrientationConstruction", resultsTuple));
    MaceLog::bgWhite("\n\nUnit test 1 outputs: ");
    for(auto str : testOutputs) {
        MaceLog::Info(str);
    }
    testOutputs.clear();


    // Unit test 2:
    success = test_EulerReconstruction(rtnStr, testOutputs);
    resultsTuple = std::make_tuple(success, rtnStr, testOutputs);
    resultsMap.insert(std::make_pair("EulerReconstruction", resultsTuple));
    MaceLog::bgWhite("\n\nUnit test 2 outputs: ");
    for(auto str : testOutputs) {
        MaceLog::Info(str);
    }
    testOutputs.clear();

    // Unit test 3:
    success = test_EulerArithmetic(rtnStr, testOutputs);
    resultsTuple = std::make_tuple(success, rtnStr, testOutputs);
    resultsMap.insert(std::make_pair("EulerArithmetic", resultsTuple));
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

#endif // UNITTESTS_ORIENTATION_H
