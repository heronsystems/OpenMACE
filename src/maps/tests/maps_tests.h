#ifndef MAPS_TESTS_H
#define MAPS_TESTS_H

#include <map>
#include <unordered_map>
#include <string>
#include <vector>
#include <tuple>

#include <maps/occupancy_definition.h>
#include <maps/data_2d_grid.h>
#include <common/logging/macelog.h>
#include <layered_map.h>


// ********************************* //
// ** Unit test 1 -- Insert Layer ** //
// ********************************* //
inline bool test_InsertLayer(std::string &returnString, std::vector<std::string> &testOutputs) {
    mace::maps::LayeredMap layeredMapTest;
    mace::maps::OccupiedResult fillValue = mace::maps::OccupiedResult::NOT_OCCUPIED;
    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* m_Map = new mace::maps::Data2DGrid<mace::maps::OccupiedResult>(&fillValue);

    // Insert 1st layer:
    layeredMapTest.updateMapLayer("occupancyLayer_1", m_Map);
    // Insert 2nd layer:
    layeredMapTest.updateMapLayer("occupancyLayer_2", m_Map);
    // Insert 3rd layer:
    layeredMapTest.updateMapLayer("occupancyLayer_3", m_Map);

    bool success = false;
    returnString = "Unit test 1 -- Insert Layer: FAIL";
    if(layeredMapTest.getLayeredMap().size() == 3) {
        success = true;
        returnString = "Unit test 1 -- Insert Layer: PASS";
    }
    return success;
}


// ************************************ //
// ** Unit test 2 -- Get Layer grid size ** //
// ************************************ //
inline bool test_GetLayerGridSize(std::string &returnString, std::vector<std::string> &testOutputs) {
    mace::maps::LayeredMap layeredMapTest;
    mace::maps::OccupiedResult fillValue = mace::maps::OccupiedResult::NOT_OCCUPIED;
    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* m_Map = new mace::maps::Data2DGrid<mace::maps::OccupiedResult>(&fillValue);
    // Insert 1st layer:
    layeredMapTest.updateMapLayer("occupancyLayer_1", m_Map);
    // Insert 2nd layer:
    layeredMapTest.updateMapLayer("occupancyLayer_2", m_Map);
    // Insert 3rd layer:
    layeredMapTest.updateMapLayer("occupancyLayer_3", m_Map);

    // Check layers size:
    double xSize, ySize, xRes, yRes;
    layeredMapTest.getLayerSize(xSize, ySize, xRes, yRes);
    std::string tmpStr = "(" + std::to_string(xSize) + ", " + std::to_string(ySize) + ", " + std::to_string(xRes) + ", " + std::to_string(yRes) + ")";
    testOutputs.push_back("Map layer size: " + tmpStr);

    bool success = false;
    returnString = "Unit test 2 -- Layer grid size: FAIL";
    if(xSize == 40 && ySize == 40 && xRes == 0.5 && yRes == 0.5) {
        success = true;
        returnString = "Unit test 2 -- Layer grid size: PASS";
    }

    return success;
}


// ************************************ //
// ** Unit test 3 -- Update grid size ** //
// ************************************ //
inline bool test_UpdateGridSize(std::string &returnString, std::vector<std::string> &testOutputs) {
    mace::maps::LayeredMap layeredMapTest;
    mace::maps::OccupiedResult fillValue = mace::maps::OccupiedResult::NOT_OCCUPIED;
    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* m_Map = new mace::maps::Data2DGrid<mace::maps::OccupiedResult>(&fillValue);
    // Insert 1st layer:
    layeredMapTest.updateMapLayer("occupancyLayer_1", m_Map);
    // Insert 2nd layer:
    layeredMapTest.updateMapLayer("occupancyLayer_2", m_Map);
    // Insert 3rd layer:
    layeredMapTest.updateMapLayer("occupancyLayer_3", m_Map);

    // Update grid size:
    layeredMapTest.updateGridSize(-20, 20, -20, 20, 0.5, 0.5);

    // Re-check layers size:
    double xSize, ySize, xRes, yRes;
    layeredMapTest.getLayerSize(xSize, ySize, xRes, yRes);
    std::string tmpStr = "(" + std::to_string(xSize) + ", " + std::to_string(ySize) + ", " + std::to_string(xRes) + ", " + std::to_string(yRes) + ")";
    testOutputs.push_back("Map layer size: " + tmpStr);

    bool success = false;
    returnString = "Unit test 3 -- Update grid size: FAIL";
    if(xSize == 80 && ySize == 80 && xRes == 0.5 && yRes == 0.5) {
        success = true;
        returnString = "Unit test 3 -- Update grid size: PASS";
    }

    return success;
}



// ********************************** //
// ** Unit test 4 -- Get map layer ** //
// ********************************** //
inline bool test_GetMapLayer(std::string &returnString, std::vector<std::string> &testOutputs) {
    mace::maps::LayeredMap layeredMapTest;
    mace::maps::OccupiedResult fillValue = mace::maps::OccupiedResult::NOT_OCCUPIED;
    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* m_Map = new mace::maps::Data2DGrid<mace::maps::OccupiedResult>(&fillValue);
    // Insert 1st layer:
    layeredMapTest.updateMapLayer("occupancyLayer_1", m_Map);
    // Insert 2nd layer:
    layeredMapTest.updateMapLayer("occupancyLayer_2", m_Map);
    // Insert 3rd layer:
    layeredMapTest.updateMapLayer("occupancyLayer_3", m_Map);

    // Get map layer (not first one):
    //    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* layer = layeredMapTest.getMapLayer("occupancyLayer_3");
    mace::maps::BaseGridMap* layer = layeredMapTest.getMapLayer("occupancyLayer_3");
    double xLength = layer->getXLength();
    double yLength = layer->getYLength();
    testOutputs.push_back("Occupancy Layer 3 x length: " + std::to_string(xLength) + "; and y length: " + std::to_string(yLength));

    bool success = false;
    returnString = "Unit test 4 -- Get map layer: FAIL";
    if(xLength == 20 && yLength == 20) {
        success = true;
        returnString = "Unit test 4 -- Get map layer: PASS";
    }

    return success;
}

// **************************************** //
// ** Unit test 5 -- UpdateLayerGridSize ** //
// **************************************** //
inline bool test_UpdateLayerGridSize(std::string &returnString, std::vector<std::string> &testOutputs) {
    mace::maps::LayeredMap layeredMapTest;
    mace::maps::OccupiedResult fillValue = mace::maps::OccupiedResult::NOT_OCCUPIED;
    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* m_Map = new mace::maps::Data2DGrid<mace::maps::OccupiedResult>(&fillValue);
    // Insert 1st layer:
    layeredMapTest.updateMapLayer("occupancyLayer_1", m_Map);
    // Insert 2nd layer:
    layeredMapTest.updateMapLayer("occupancyLayer_2", m_Map);
    // Insert 3rd layer:
    layeredMapTest.updateMapLayer("occupancyLayer_3", m_Map);

    // Perform operation on map layer (resize -- callback should fire for all others):
    mace::maps::BaseGridMap* layer = layeredMapTest.getMapLayer("occupancyLayer_3");
    layer->updateGridSize(-30, 30, -30, 30, 0.5, 0.5);

    // Re-check layers size of the layered map layers:
    double xSize, ySize, xRes, yRes;
    layeredMapTest.getLayerSize(xSize, ySize, xRes, yRes);
    std::string tmpStr = "(" + std::to_string(xSize) + ", " + std::to_string(ySize) + ", " + std::to_string(xRes) + ", " + std::to_string(yRes) + ")";
    testOutputs.push_back("Map layer size: " + tmpStr);

    bool success = false;
    returnString = "Unit test 5 -- Update layer grid size: FAIL";
    if(xSize == 120 && ySize == 120 && xRes == 0.5 && yRes == 0.5) {
        success = true;
        returnString = "Unit test 5 -- Update layer grid size: PASS";
    }

    return success;
}


// **************************************** //
// ** Unit test 6 -- Set grid resolution ** //
// **************************************** //
inline bool test_SetGridResolution(std::string &returnString, std::vector<std::string> &testOutputs) {
    mace::maps::LayeredMap layeredMapTest;
    mace::maps::OccupiedResult fillValue = mace::maps::OccupiedResult::NOT_OCCUPIED;
    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* m_Map = new mace::maps::Data2DGrid<mace::maps::OccupiedResult>(&fillValue);
    // Insert 1st layer:
    layeredMapTest.updateMapLayer("occupancyLayer_1", m_Map);
    // Insert 2nd layer:
    layeredMapTest.updateMapLayer("occupancyLayer_2", m_Map);
    // Insert 3rd layer:
    layeredMapTest.updateMapLayer("occupancyLayer_3", m_Map);

    double xRes, yRes;
    xRes = layeredMapTest.getXResolution();
    yRes = layeredMapTest.getYResolution();
    // Update grid resolution:
    layeredMapTest.setXResolution(1.0);
    layeredMapTest.setYResolution(1.0);
    double xRes_new = layeredMapTest.getXResolution();
    double yRes_new = layeredMapTest.getYResolution();

    bool success = false;
    returnString = "Unit test 6 -- Set grid resolution: FAIL";
    if(xRes == 0.5 && yRes == 0.5 && xRes_new == 1.0 && yRes_new == 1.0) {
        success = true;
        returnString = "Unit test 6 -- Set grid resolution: PASS";
    }

    return success;
}


// ***************************************************** //
// ***** Unit test 7 -- Rotate layers about Origin ***** //
// ***************************************************** //
inline bool test_RotateLayers(std::string &returnString, std::vector<std::string> &testOutputs) {
    mace::maps::LayeredMap layeredMapTest;
    mace::maps::OccupiedResult fillValue = mace::maps::OccupiedResult::NOT_OCCUPIED;
    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* m_Map = new mace::maps::Data2DGrid<mace::maps::OccupiedResult>(&fillValue);
    // Insert 1st layer:
    layeredMapTest.updateMapLayer("occupancyLayer_1", m_Map);
    // Insert 2nd layer:
    layeredMapTest.updateMapLayer("occupancyLayer_2", m_Map);
    // Insert 3rd layer:
    layeredMapTest.updateMapLayer("occupancyLayer_3", m_Map);


    // Get starting angle:
    double startingAngle = layeredMapTest.getRotationAngleDegrees();
    testOutputs.push_back("Map layer starting angle: " + std::to_string(startingAngle));

    // Get a point at (1, 0):
    //    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* layer = layeredMapTest.getMapLayer("occupancyLayer_3");
    mace::maps::BaseGridMap* layerBeforeRot = layeredMapTest.getMapLayer("occupancyLayer_3");
    double xPos, yPos;
    layerBeforeRot->getPositionFromIndex(0, xPos, yPos);
    testOutputs.push_back("Index 0 (x,y): (" + std::to_string(xPos) + ", " + std::to_string(yPos) + ")");

    // Set rotation angle to 90 degrees:
    layeredMapTest.setRotationAngleDegrees(90);

    // Get ending angle:
    double endingAngle = layeredMapTest.getRotationAngleDegrees();
    testOutputs.push_back("Map layer ending angle: " + std::to_string(endingAngle));

    // Get rotated point:
    //    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* layer = layeredMapTest.getMapLayer("occupancyLayer_3");
    mace::maps::BaseGridMap* layerAfterRot = layeredMapTest.getMapLayer("occupancyLayer_2");
    double xPosRot, yPosRot;
    layerAfterRot->getPositionFromIndex(0, xPosRot, yPosRot);
    testOutputs.push_back("Index 0 rotated (x,y): (" + std::to_string(xPosRot) + ", " + std::to_string(yPosRot) + ")");

    // Set rotation angle back to 0 degrees:
    layeredMapTest.setRotationAngleDegrees(00);

    // Get final angle:
    double finalAngle = layeredMapTest.getRotationAngleDegrees();
    testOutputs.push_back("Map layer final angle: " + std::to_string(finalAngle));

    // Get rotated point:
    //    mace::maps::Data2DGrid<mace::maps::OccupiedResult>* layer = layeredMapTest.getMapLayer("occupancyLayer_3");
    mace::maps::BaseGridMap* layerAfterRotBack = layeredMapTest.getMapLayer("occupancyLayer_1");
    double xPosBack, yPosBack;
    layerAfterRotBack->getPositionFromIndex(0, xPosBack, yPosBack);
    testOutputs.push_back("Index 0 rotated back (x,y): (" + std::to_string(xPosBack) + ", " + std::to_string(yPosBack) + ")");


    bool success = false;
    returnString = "Unit test 7 -- Rotate layers: FAIL";
    if(startingAngle == 0.0 && endingAngle == 90 && finalAngle == 0.0 &&
            xPos == -10 && yPos == -10 &&
            xPosRot == 10 && yPosRot == -10 &&
            xPosBack == -10 && yPosBack == -10) {
        success = true;
        returnString = "Unit test 7 -- Rotate layers: PASS";
    }

    return success;
}



// **************************************** //
// ************* TEST RESULTS ************* //
// **************************************** //
inline void runMapsTests() {
    // Set up variables:
    std::vector<std::tuple<std::string, bool, std::string, std::vector<std::string>>> resultsVector;
    std::string rtnStr;
    std::vector<std::string> testOutputs;
    bool success;

    // Unit test 1:
    testOutputs.clear();
    success = test_InsertLayer(rtnStr, testOutputs);
    std::string testName1 = "InsertLayer";
    std::tuple<std::string, bool, std::string, std::vector<std::string>> resultsTuple1 = std::make_tuple(testName1, success, rtnStr, testOutputs);
    resultsVector.push_back(resultsTuple1);
    MaceLog::bgWhite("\n\nUnit test 1 outputs: ");
    for(auto str : testOutputs) {
        MaceLog::Info(str);
    }

    // Unit test 2:
    testOutputs.clear();
    success = test_GetLayerGridSize(rtnStr, testOutputs);
    std::string testName2 = "GetLayerGridSize";
    std::tuple<std::string, bool, std::string, std::vector<std::string>> resultsTuple2 = std::make_tuple(testName2, success, rtnStr, testOutputs);
    resultsVector.push_back(resultsTuple2);
    MaceLog::bgWhite("\n\nUnit test 2 outputs: ");
    for(auto str : testOutputs) {
        MaceLog::Info(str);
    }

    // Unit test 3:
    testOutputs.clear();
    success = test_UpdateGridSize(rtnStr, testOutputs);
    std::string testName3 = "UpdateGridSize";
    std::tuple<std::string, bool, std::string, std::vector<std::string>> resultsTuple3 = std::make_tuple(testName3, success, rtnStr, testOutputs);
    resultsVector.push_back(resultsTuple3);
    MaceLog::bgWhite("\n\nUnit test 3 outputs: ");
    for(auto str : testOutputs) {
        MaceLog::Info(str);
    }

    // Unit test 4:
    testOutputs.clear();
    success = test_GetMapLayer(rtnStr, testOutputs);
    std::string testName4 = "GetMapLayer";
    std::tuple<std::string, bool, std::string, std::vector<std::string>> resultsTuple4 = std::make_tuple(testName4, success, rtnStr, testOutputs);
    resultsVector.push_back(resultsTuple4);
    MaceLog::bgWhite("\n\nUnit test 4 outputs: ");
    for(auto str : testOutputs) {
        MaceLog::Info(str);
    }

    // Unit test 5:
    testOutputs.clear();
    success = test_UpdateLayerGridSize(rtnStr, testOutputs);
    std::string testName5 = "UpdateLayerGridSize";
    std::tuple<std::string, bool, std::string, std::vector<std::string>> resultsTuple5 = std::make_tuple(testName3, success, rtnStr, testOutputs);
    resultsVector.push_back(resultsTuple5);
    MaceLog::bgWhite("\n\nUnit test 5 outputs: ");
    for(auto str : testOutputs) {
        MaceLog::Info(str);
    }

    // Unit test 6:
    testOutputs.clear();
    success = test_SetGridResolution(rtnStr, testOutputs);
    std::string testName6 = "SetGridResolution";
    std::tuple<std::string, bool, std::string, std::vector<std::string>> resultsTuple6 = std::make_tuple(testName6, success, rtnStr, testOutputs);
    resultsVector.push_back(resultsTuple6);
    MaceLog::bgWhite("\n\nUnit test 6 outputs: ");
    for(auto str : testOutputs) {
        MaceLog::Info(str);
    }

    // Unit test 7:
    testOutputs.clear();
    success = test_RotateLayers(rtnStr, testOutputs);
    std::string testName7 = "RotateLayers";
    std::tuple<std::string, bool, std::string, std::vector<std::string>> resultsTuple7 = std::make_tuple(testName7, success, rtnStr, testOutputs);
    resultsVector.push_back(resultsTuple7);
    MaceLog::bgWhite("\n\nUnit test 7 outputs: ");
    for(auto str : testOutputs) {
        MaceLog::Info(str);
    }


    // Print pass/fail results
    MaceLog::bgWhite("\n\nTest Results: ");
    for(auto test : resultsVector) {
        if(std::get<1>(test)) {
            MaceLog::Green(std::get<2>(test));
        }
        else {
            MaceLog::Red(std::get<2>(test));
        }
    }

}

#endif // MAPS_TESTS_H
