#include <iostream>

#include "graphs/graph.h"

#include <unordered_map>
#include <set>

#include "graphs/road_network.h"

using namespace mace::graphs;

// Quick test for boost graph library wrapper

std::string PosToString(mace::state_space::StatePtr pos)
{
    auto posCast3DC = dynamic_cast<mace::pose::CartesianPosition_3D *>(pos.get());
    if (posCast3DC)
    {
        std::string result = "(" + std::to_string(posCast3DC->getXPosition()) + ", "
                + std::to_string(posCast3DC->getYPosition()) + ", "
                + std::to_string(posCast3DC->getZPosition()) + ", Cartesian)";

        return result;
    }

    auto posCast2DC  = dynamic_cast<mace::pose::CartesianPosition_2D *>(pos.get());
    if (posCast2DC)
    {
        std::string result = "(" + std::to_string(posCast2DC->getXPosition()) + ", "
                + std::to_string(posCast2DC->getYPosition()) + ", Cartesian)";

        return result;
    }

    auto posCast3DG = dynamic_cast<mace::pose::GeodeticPosition_3D *>(pos.get());
    if (posCast3DG)
    {
        std::string result = "(" + std::to_string(posCast3DG->getLatitude()) + ", "
                + std::to_string(posCast3DG->getLongitude()) + ", "
                + std::to_string(posCast3DG->getAltitude()) + ", Geodetic)";

        return result;
    }

    auto posCast2DG  = dynamic_cast<mace::pose::GeodeticPosition_2D *>(pos.get());
    if (posCast2DG)
    {
        std::string result = "Geodetic: (" + std::to_string(posCast2DG->getLatitude()) + ", "
                + std::to_string(posCast2DG->getLongitude()) + ", Geodetic)";

        return result;
    }

    return std::string("");
};

void TestAddNodes(RoadNetwork &testNetwork,
                  std::unordered_map<int, mace::state_space::StatePtr> &positions,
                  std::unordered_map<int, RoadNetwork::NodeType> &states,
                  std::unordered_map<int, bool> &isOpen)
{
    bool success = true;
    std::cout << "Adding nodes to road network... ";

    for (int i = 1; i <= positions.size(); ++i)
    {
        if (!testNetwork.AddNode(positions.at(i), states.at(i), isOpen.at(i)))
        {
            success = false;
            std::cout << "FAIL" << std::endl;
            std::cout << "Failed to add node " << i << " with position " << positions.at(i) << std::endl;
            break;
        }
    }
    if (success)
        std::cout << "PASS" << std::endl;
}

void TestAddEdges(RoadNetwork &testNetwork,
                  std::unordered_map<int, mace::state_space::StatePtr> &positions,
                  std::unordered_map<int, std::vector<int>> edges)
{
    bool success = true;
    std::cout << "Adding eges to road network... ";

    for (int i = 1; i <= positions.size(); ++i)
    {
        for (int j = 0; j < edges.at(i).size(); ++j)
        {
            if (!testNetwork.AddEdge(positions.at(i), positions.at(edges.at(i).at(j))))
            {
                success = false;
                std::cout << "FAIL" << std::endl;
                std::cout << "Failed to add edge (" << i << ", " << edges.at(i).at(j) << ")" << std::endl;
                break;
            }
        }
        if (!success)
            break;
    }
    if (success)
        std::cout << "PASS" << std::endl;
}

void TestConnectivity(RoadNetwork &testNetwork,
                      std::unordered_map<int, mace::state_space::StatePtr> &positions,
                      std::unordered_map<int, std::set<int>> &connectivity)
{
    bool success = true;
    std::cout << "Testing connected/unconnected graph functionality... ";

    bool expected;
    for (int i = 1; i <= positions.size(); ++i)
    {
        for (int j = 1; j <= positions.size(); ++j)
        {
            expected = connectivity.at(i).find(j) != connectivity.at(i).cend();
            if (testNetwork.Connected(positions.at(i), positions.at(j)) != expected)
            {
                success = false;
                std::cout << "FAIL" << std::endl;
                if (expected)
                    std::cout << "Nodes " << i << " and " << j << " are supposed to be connected" << std::endl;
                else
                    std::cout << "Nodes " << i << " and " << j << " are supposed to be disconnected" << std::endl;
            }
        }
    }
    if (success)
        std::cout << "PASS" << std::endl;
}

void TestPositionEquality(RoadNetwork &testNetwork,
                          std::unordered_map<int, mace::state_space::StatePtr> &positions,
                          std::unordered_map<int, mace::state_space::StatePtr> &altPositions,
                          std::vector<bool> &expectedEqualityResults)
{
    bool success = true;
    std::cout << "Testing node equality... ";
    for (int i = 1; i <= altPositions.size(); ++i)
    {
        if (testNetwork.NodeExists(altPositions.at(i)) != expectedEqualityResults.at(i - 1))
        {
            success = false;
            std::cout << "FAIL" << std::endl;
            if (expectedEqualityResults.at(i - 1))
                std::cout << "Positions " << PosToString(positions.at(i))
                          << " and " << PosToString(altPositions.at(i))
                          << "should be considered equal" << std::endl;
            else
                std::cout << "Positions " << PosToString(positions.at(i))
                          << " and " << PosToString(altPositions.at(i))
                          << "should be considered unequal" << std::endl;

            break;
        }
    }
    if (success)
        std::cout << "PASS" << std::endl;
}

void TestNodeUpdate(RoadNetwork &testNetwork,
                    std::unordered_map<int, mace::state_space::StatePtr> &positions)
{
    bool success = true;
    std::cout << "Testing update of node data... ";

    RoadNetwork::NodeType prevState, resultState;

    testNetwork.GetNodeType(positions.at(1), prevState);

    RoadNetwork::NodeType changeState = RoadNetwork::NodeType::Unexplored;
    if (prevState == changeState)
        changeState = RoadNetwork::NodeType::Obstacle;

    testNetwork.SetNodeType(positions.at(1), changeState);
    testNetwork.GetNodeType(positions.at(1), resultState);
    if (resultState != changeState)
    {
        success = false;
        std::cout << "FAIL" << std::endl;
        std::cout << "Failed to change node state" << std::endl;
    }

    testNetwork.SetNodeType(positions.at(1), prevState);

    if (success)
        std::cout << "PASS" << std::endl;
}

void TestBFS(RoadNetwork &testNetwork,
             std::unordered_map<int, mace::state_space::StatePtr> &positions,
             std::unordered_map<int, bool> isOpen,
             std::unordered_map<mace::state_space::State *, int> &positionsToID,
             std::unordered_map<int, std::set<int>> &connectivity)
{
    bool success = true;
    std::cout << "Testing breadth-first search for unexplored nodes... ";


    std::set<int> expectedResults;

    std::vector<mace::state_space::StatePtr> results;

    for (int nodeIndex = 1; nodeIndex <= positions.size(); ++nodeIndex)
    {
        expectedResults.clear();
        for (int j : connectivity.at(nodeIndex))
        {
            if (isOpen.at(j))
                expectedResults.insert(j);
        }

        results.clear();
        testNetwork.FindOpenNodesBFS(positions.at(nodeIndex), results);

        if (results.size() != expectedResults.size())
        {
            success = false;
            std::cout << "FAIL" << std::endl;
            std::cout << "Expected " << expectedResults.size() << " results searching from node #" << nodeIndex << " but got " << results.size() << std::endl;
            break;
        }

        std::set<int> seen;
        for (const mace::state_space::StatePtr &pos : results)
        {
            int nextID = positionsToID.at(pos.get());
            if (seen.find(nextID) != seen.cend())
            {
                success = false;
                std::cout << "FAIL" << std::endl;
                std::cout << "Node #" << nextID << " was found twice, searching from node #" << nodeIndex << std::endl;

                break;
            }
            if (expectedResults.find(nextID) == expectedResults.cend())
            {
                success = false;
                std::cout << "FAIL" << std::endl;
                std::cout << "Node #" << nextID << " should not have been found, searching from node #" << nodeIndex << std::endl;

                break;
            }
        }

        if (!success)
            break;
    }

    if (success)
        std::cout << "PASS" << std::endl;
}

void TestDFS(RoadNetwork &testNetwork,
             std::unordered_map<int, mace::state_space::StatePtr> &positions,
             std::unordered_map<int, bool> &isOpen,
             std::unordered_map<mace::state_space::State *, int> &positionsToID,
             std::unordered_map<int, std::set<int>> &connectivity,
             bool connectedOnly)
{
    bool success = true;
    std::set<int> expectedResults;
    if (connectedOnly)
        std::cout << "Testing depth-first search for unexplored nodes... ";
    else
        std::cout << "Testing search for unexplored nodes, ignoring connectivity... ";

    auto test = [&](int nodeIndex, std::set<int> expected)
    {
        std::vector<mace::state_space::StatePtr> results;
        testNetwork.FindOpenNodesDFS(positions.at(nodeIndex), results, connectedOnly);

        if (results.size() != expected.size())
        {
            success = false;
            std::cout << "FAIL" << std::endl;
            std::cout << "Expected " << expected.size() << " results searching from node #" << nodeIndex << " but got " << results.size() << std::endl;
            return;
        }

        std::set<int> seen;
        for (const mace::state_space::StatePtr &pos : results)
        {
            int nextID = positionsToID.at(pos.get());
            if (seen.find(nextID) != seen.cend())
            {
                success = false;
                std::cout << "FAIL" << std::endl;
                std::cout << "Node #" << nextID << " was found twice, searching from node #" << nodeIndex << std::endl;

                return;
            }
            if (expected.find(nextID) == expected.cend())
            {
                success = false;
                std::cout << "FAIL" << std::endl;
                std::cout << "Node #" << nextID << " should not have been found, searching from node #" << nodeIndex << std::endl;

                return;
            }
        }
    };

    if (connectedOnly)
    {
        for (int i = 1; i <= positions.size(); ++i)
        {
            expectedResults.clear();
            for (int j : connectivity.at(i))
            {
                if (isOpen.at(j))
                    expectedResults.insert(j);
            }
            test(i, expectedResults);

            if (!success)
                break;
        }
    }
    else
    {
        for (int i = 1; i <= positions.size(); ++i)
        {
            if (isOpen.at(i))
                expectedResults.insert(i);
        }

        for(int i = 1; i <= positions.size(); ++i)
        {
            test(i, expectedResults);

            if (!success)
                break;
        }
    }

    if (success)
        std::cout << "PASS" << std::endl;
}

void TestOriginChange(RoadNetwork &testNetwork,
                      mace::pose::GeodeticPosition_3D &origin,
                      mace::pose::GeodeticPosition_3D &altOrigin,
                      mace::state_space::StatePtr &position,
                      mace::state_space::StatePtr &translatedPosition) // Existing node, whose coordinates would be invalid after origin change
{
    bool success = true;
    std::cout << "Testing origin change... ";

    testNetwork.setGlobalOrigin(altOrigin);

    if (testNetwork.NodeExists(position))
    {
        success = false;
        std::cout << "FAIL" << std::endl;
        std::cout << "Detected node which shouldn't exist, after shifting global origin" << std::endl;
        return;
    }

    if (!testNetwork.NodeExists(translatedPosition))
    {
        success = false;
        std::cout << "FAIL" << std::endl;
        std::cout << "Failed to detected existing node, after shifting global origin" << std::endl;
        return;
    }

    testNetwork.setGlobalOrigin(origin);

    if (!testNetwork.NodeExists(position))
    {
        success = false;
        std::cout << "FAIL" << std::endl;
        std::cout << "Failed to detect existing node, after resetting to the original origin" << std::endl;
        return;
    }

    if (!testNetwork.NodeExists(position))
    {
        success = false;
        std::cout << "FAIL" << std::endl;
        std::cout << "Detected node which shouldn't exist, after resetting global origin" << std::endl;
        return;
    }

    if (success)
        std::cout << "PASS" << std::endl;
}

void TestNeighbors(RoadNetwork &testNetwork,
                   std::unordered_map<int, mace::state_space::StatePtr> &positions,
                   std::unordered_map<mace::state_space::State *, int> &positionsToID,
                   std::unordered_map<int, std::vector<int>> edges)
{
    bool success = true;
    std::cout << "Testing adjacent nodes... ";
    for (auto idPositionPair : positions)
    {
        std::unordered_set<int> expected;
        for (auto edgePair : edges)
        {
            for (int node : edgePair.second)
            {
                if (edgePair.first == idPositionPair.first)
                    expected.insert(node);
                else if (node == idPositionPair.first)
                {
                    expected.insert(edgePair.first);
                    break;
                }
            }
        }
        auto results = testNetwork.GetAdjacentNodes(idPositionPair.second);

        if (expected.size() != results.size())
        {
            success = false;
            std::cout << "FAIL" << std::endl;
            std::cout << "Expected " << expected.size() << " adjacent nodes for node #" << idPositionPair.first << " but got " << results.size() << std::endl;
            break;
        }

        for (auto result : results)
        {
            int resultID = positionsToID.at(result.first.get());
            if (expected.find(resultID) == expected.cend())
            {
                success = false;
                std::cout << "FAIL" << std::endl;
                std::cout << "Node #" << resultID << " is not adjacent to node #" << idPositionPair.first << std::endl;
                break;
            }
        }

        if (!success)
            break;
    }

    if (success)
        std::cout << "PASS" << std::endl;
}

void TestFindConnectedNodes(RoadNetwork &testNetwork,
                            std::unordered_map<int, mace::state_space::StatePtr> &positions,
                            std::unordered_map<mace::state_space::State *, int> &positionsToID,
                            std::unordered_map<int, std::set<int>> connectivity)
{
    bool success = true;
    std::cout << "Testing find connected nodes... ";
    for (auto idPositionPair : positions)
    {
        auto results = testNetwork.GetConnectedNodes(idPositionPair.second);

        auto &expected = connectivity.at(idPositionPair.first);

        if (expected.size() != results.size())
        {
            success = false;
            std::cout << "FAIL" << std::endl;
            std::cout << "Expected " << expected.size() << " connected nodes for node #" << idPositionPair.first << " but got " << results.size() << std::endl;
            break;
        }

        for (auto result : results)
        {
            int resultID = positionsToID.at(result.first.get());
            if (expected.find(resultID) == expected.cend())
            {
                success = false;
                std::cout << "FAIL" << std::endl;
                std::cout << "Node #" << resultID << " is not adjacent to node #" << idPositionPair.first << std::endl;
                break;
            }
        }

        if (!success)
            break;
    }

    if (success)
        std::cout << "PASS" << std::endl;
}


void TestIncrementalSearch(RoadNetwork &testNetwork,
                           std::unordered_map<int, mace::state_space::StatePtr> &positions,
                           std::unordered_map<mace::state_space::State *, int> &positionsToID,
                           std::unordered_map<int, RoadNetwork::NodeType> &states,
                           std::unordered_map<int, bool> &isOpen,
                           std::unordered_map<int, std::vector<int>> &edges,
                           std::unordered_map<int, std::set<int>> &connectivity)
{
    bool success = true;
    std::cout << "Testing incremental search... ";


    positions.insert({-1, std::make_shared<mace::pose::CartesianPosition_2D>(1000, -1000)});
    positions.insert({-2, std::make_shared<mace::pose::CartesianPosition_2D>(1000, 1000)});
    positions.insert({-3, std::make_shared<mace::pose::CartesianPosition_2D>(-1000, 1000)});
    positions.insert({-4, std::make_shared<mace::pose::CartesianPosition_2D>(-1000, -1000)});

    for (int i = -1; i >= -4; --i)
        positionsToID.insert({positions.at(i).get(), i});

    states.insert({-1, RoadNetwork::NodeType::Unexplored});
    states.insert({-2, RoadNetwork::NodeType::IsRoad});
    states.insert({-3, RoadNetwork::NodeType::IsRoad});
    states.insert({-4, RoadNetwork::NodeType::Unexplored});

    isOpen.insert({-1, true});
    isOpen.insert({-2, false});
    isOpen.insert({-3, false});
    isOpen.insert({-4, true});

    edges.insert({-4, {-3}});
    edges.insert({-3, {-2}});
    edges.insert({-2, {-1}});
    edges.insert({-1, {1}});

    connectivity.insert({-1, {-1, -2, -3, -4}});
    connectivity.insert({-2, {-1, -2, -3, -4}});
    connectivity.insert({-3, {-1, -2, -3, -4}});
    connectivity.insert({-4, {-1, -2, -3, -4}});

    for (int index : connectivity.at(1))
    {
        for (int i = -1; i >= -4; --i)
            connectivity.at(i).insert(index);

        connectivity.at(index).insert(-1);
        connectivity.at(index).insert(-2);
        connectivity.at(index).insert(-3);
        connectivity.at(index).insert(-4);
    }

    testNetwork.Init_IncrementalFindOpenNodesBFS();
    std::vector<mace::state_space::StatePtr> results;
    testNetwork.IncrementalFindOpenNodes(positions.at(1), results);

    testNetwork.AddNode(positions.at(-1), states.at(-1), isOpen.at(-1));
    testNetwork.AddNode(positions.at(-2), states.at(-2), isOpen.at(-2));

    testNetwork.IncrementalFindOpenNodes(positions.at(1), results);
    if (results.size() != 0)
    {
        success = false;
        std::cout << "FAIL" << std::endl;
        std::cout << "Incremental BFS found " << results.size() << " results, but expected 0 (added nodes but no edges)" << std::endl;
    }

    testNetwork.AddEdge(positions.at(1), positions.at(-1));
    testNetwork.AddEdge(positions.at(-1), positions.at(-2));

    testNetwork.IncrementalFindOpenNodes(positions.at(1), results);
    if (results.size() != 1)
    {
        success = false;
        std::cout << "FAIL" << std::endl;
        std::cout << "Incremental BFS found " << results.size() << " results, but expected 1 (added nodes and edges)" << std::endl;

        for (auto &result : results)
            std::cout << PosToString(result) << std::endl;
    }

    results.clear();
    testNetwork.Init_IncrementalFindOpenNodesDFS();
    testNetwork.IncrementalFindOpenNodes(positions.at(1), results);

    testNetwork.AddNode(positions.at(-3), states.at(-3), isOpen.at(-3));
    testNetwork.AddNode(positions.at(-4), states.at(-4), isOpen.at(-4));

    testNetwork.IncrementalFindOpenNodes(positions.at(1), results);
    if (results.size() != 0)
    {
        success = false;
        std::cout << "FAIL" << std::endl;
        std::cout << "Incremental DFS found " << results.size() << " results, but expected 0 (added nodes but no edges)" << std::endl;
    }

    testNetwork.AddEdge(positions.at(-2), positions.at(-3));
    testNetwork.AddEdge(positions.at(-3), positions.at(-4));

    testNetwork.IncrementalFindOpenNodes(positions.at(1), results);
    if (results.size() != 1)
    {
        success = false;
        std::cout << "FAIL" << std::endl;
        std::cout << "Incremental DFS found " << results.size() << " results, but expected 1 (added nodes and edges)" << std::endl;
    }


    if (success)
        std::cout << "PASS" << std::endl;
}

void TestPropogateData(RoadNetwork &testNetwork,
                       std::unordered_map<int, mace::state_space::StatePtr> &positions,
                       std::unordered_map<int, bool> &isOpen,
                       std::unordered_map<int, std::set<int>> &connectivity)
{
    bool success = true;
    std::cout << "Testing data propogation by updating whether nodes are open... ";
    auto lambda = [&](RoadNetwork::NodeData &data)
    {
        data.isOpen = true;
        return false;
    };

    testNetwork.PropogateDataBFS(positions.at(1), lambda);
    std::vector<mace::state_space::StatePtr> results;
    testNetwork.FindOpenNodesBFS(positions.at(1), results);

    if (results.size() != connectivity.at(1).size())
    {
        success = false;
        std::cout << "Data failed to propogate correctly using BFS, found " << results.size() << " unexplored nodes but expected " << connectivity.at(1).size() << std::endl;
    }
    else
    {
        for (auto idPositionPair : positions) // reset states
            testNetwork.SetNodeOpen(idPositionPair.second, isOpen.at(idPositionPair.first));

        results.clear();

        testNetwork.PropogateDataDFS(positions.at(1), lambda);
        testNetwork.FindOpenNodesBFS(positions.at(1), results);

        if (results.size() != connectivity.at(1).size())
        {
            success = false;
            std::cout << "Data failed to propogate correctly using DFS, found " << results.size() << " unexplored nodes but expected " << connectivity.at(1).size() << std::endl;
        }
    }

    for (auto idPositionPair : positions) // reset states
        testNetwork.SetNodeOpen(idPositionPair.second, isOpen.at(idPositionPair.first));

    if (success)
        std::cout << "PASS" << std::endl;
}

int main()
{
    mace::pose::GeodeticPosition_3D origin(0.0, 0.0, 0.0);
    mace::pose::GeodeticPosition_3D altOrigin;
    mace::pose::CartesianPosition_3D altOriginCartesian(5,-5,1);

    mace::pose::DynamicsAid::LocalPositionToGlobal(origin, altOriginCartesian, altOrigin);

    std::unordered_map<int, mace::state_space::StatePtr> positions;
    std::unordered_map<int, mace::state_space::StatePtr> altPositions;
    std::unordered_map<mace::state_space::State *, int> positionsToID; // Maps pointer back to integer ID
    std::unordered_map<int, RoadNetwork::NodeType> types;
    std::unordered_map<int, bool> isOpen;
    std::unordered_map<int, std::vector<int>> edges;
    std::unordered_map<int, std::set<int>> connectivity;


    positions.insert({1, std::make_shared<mace::pose::CartesianPosition_2D>(1, -1)});
    positions.insert({2, std::make_shared<mace::pose::CartesianPosition_2D>(1, 0)});
    positions.insert({3, std::make_shared<mace::pose::CartesianPosition_2D>(1, 1)});
    positions.insert({4, std::make_shared<mace::pose::CartesianPosition_2D>(0, 1)});
    positions.insert({5, std::make_shared<mace::pose::CartesianPosition_2D>(-1, 1)});
    positions.insert({6, std::make_shared<mace::pose::CartesianPosition_3D>(-1, -1, 0)});
    positions.insert({7, std::make_shared<mace::pose::CartesianPosition_3D>(1, 5, 1)});
    positions.insert({8, std::make_shared<mace::pose::CartesianPosition_3D>(-1, 5, 2)});
    positions.insert({9, std::make_shared<mace::pose::GeodeticPosition_2D>(0.01, 0.01)});
    positions.insert({10, std::make_shared<mace::pose::GeodeticPosition_3D>(-0.01, 0.01, 1)});

    // Offset positions to test equality
    altPositions.insert({1, std::make_shared<mace::pose::CartesianPosition_3D>(1, -1, 0.25)});
    altPositions.insert({2, std::make_shared<mace::pose::CartesianPosition_3D>(1, 0, -0.4)});
    altPositions.insert({3, std::make_shared<mace::pose::CartesianPosition_2D>(0.9, 1.1)});
    altPositions.insert({4, std::make_shared<mace::pose::CartesianPosition_2D>(0, 1.8)});
    altPositions.insert({5, std::make_shared<mace::pose::CartesianPosition_3D>(-1, 1, 2)});
    altPositions.insert({6, std::make_shared<mace::pose::CartesianPosition_3D>(-1, -1, 3)});
    altPositions.insert({7, std::make_shared<mace::pose::CartesianPosition_3D>(1, 5, 1)});
    altPositions.insert({8, std::make_shared<mace::pose::CartesianPosition_3D>(-1, 18, 2)});
    altPositions.insert({9, std::make_shared<mace::pose::GeodeticPosition_2D>(0.01, 0.01)});
    altPositions.insert({10, std::make_shared<mace::pose::GeodeticPosition_3D>(-5.01, 5.01, 5)});

    // This is set to be node 1 translated to the alternate origin
    mace::state_space::StatePtr translatedPosition = std::make_shared<mace::pose::CartesianPosition_3D>(-4, 4, 1);

    std::vector<bool> expectedEqualityResults({true, true, true, false, false, false, true, false, true, false});

    types.insert({1, RoadNetwork::NodeType::IsRoad});
    types.insert({2, RoadNetwork::NodeType::FreeSpace});
    types.insert({3, RoadNetwork::NodeType::IsRoad});
    types.insert({4, RoadNetwork::NodeType::Unexplored});
    types.insert({5, RoadNetwork::NodeType::Unexplored});
    types.insert({6, RoadNetwork::NodeType::IsRoad});
    types.insert({7, RoadNetwork::NodeType::Obstacle});
    types.insert({8, RoadNetwork::NodeType::Unexplored});
    types.insert({9, RoadNetwork::NodeType::IsRoad});
    types.insert({10, RoadNetwork::NodeType::IsRoad});

    isOpen.insert({1, false});
    isOpen.insert({2, false});
    isOpen.insert({3, false});
    isOpen.insert({4, true});
    isOpen.insert({5, true});
    isOpen.insert({6, false});
    isOpen.insert({7, false});
    isOpen.insert({8, true});
    isOpen.insert({9, false});
    isOpen.insert({10, false});

    RoadNetwork testNetwork(origin); // Using default spacing of 1.0

    // Note: RoadNetwork is undirected, so edges are only declared from one node in the following map
    edges.insert({1, {2, 4}});
    edges.insert({2, {3, 4}});
    edges.insert({3, {}});
    edges.insert({4, {5}});
    edges.insert({5, {}});
    edges.insert({6, {7, 8}});
    edges.insert({7, {}});
    edges.insert({8, {}});
    edges.insert({9, {}});
    edges.insert({10, {}});

    // Manually set if nodes are connected
    connectivity.insert({1, {1, 2, 3, 4, 5}});
    connectivity.insert({2, {1, 2, 3, 4, 5}});
    connectivity.insert({3, {1, 2, 3, 4, 5}});
    connectivity.insert({4, {1, 2, 3, 4, 5}});
    connectivity.insert({5, {1, 2, 3, 4, 5}});
    connectivity.insert({6, {6, 7, 8}});
    connectivity.insert({7, {6, 7, 8}});
    connectivity.insert({8, {6, 7, 8}});
    connectivity.insert({9, {9}});
    connectivity.insert({10, {10}});

    for (int i = 1; i <= 10; ++i)
        positionsToID.insert({positions.at(i).get(), i});

    // Unit test 1  - Add nodes
    TestAddNodes(testNetwork, positions, types, isOpen);

    // Unit test 2  - Add edges
    TestAddEdges(testNetwork, positions, edges);

    // Unit test 3 - Check connected/unconnected functions as expected
    TestConnectivity(testNetwork, positions, connectivity);

    // Unit test 4 - Check that close positions are considered equal
    TestPositionEquality(testNetwork, positions, altPositions, expectedEqualityResults);

    // Unit test 5 - Check that node data can be updated
    TestNodeUpdate(testNetwork, positions);

    // Unit test 6 - Check that unexplored nodes are found correctly for BFS
    TestBFS(testNetwork, positions, isOpen, positionsToID, connectivity);

    // Unit test 7 - Check that unexplored nodes are found correctly for DFS
    TestDFS(testNetwork, positions, isOpen, positionsToID, connectivity, true);

    // Unit test 8 - Check that unexplored nodes are found correctly when searching full graph (ignoring connectivity)
    TestDFS(testNetwork, positions, isOpen, positionsToID, connectivity, false);

    // Unit test 9 - Check that updating global origin correctly translates new nodes
    TestOriginChange(testNetwork, origin, altOrigin, positions.at(1), translatedPosition);

    // Unit test 10 - Test neighbors
    TestNeighbors(testNetwork, positions, positionsToID, edges);

    // Unit test 11 - Test find connected nodes
    TestFindConnectedNodes(testNetwork, positions, positionsToID, connectivity);

    // Unit test 12 - Test incremental search (adding new nodes/edges)
    TestIncrementalSearch(testNetwork, positions, positionsToID, types, isOpen, edges, connectivity);

    // Unit test 13 - Test propogate data
    TestPropogateData(testNetwork, positions, isOpen, connectivity);
}
