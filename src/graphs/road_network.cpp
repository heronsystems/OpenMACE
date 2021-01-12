#include "road_network.h"

#include "data_tasks/task_loiter_descriptor.h"

namespace mace {
namespace graphs {

/*!
 * \brief Constructor
 * \param globalOrigin Global origin
 * \param nodeSpacing Node spacing. Each possible node is a cube with this width
 */
RoadNetwork::RoadNetwork(const mace::pose::GeodeticPosition_3D &globalOrigin, double nodeSpacing) :
    m_globalOrigin(globalOrigin),
    m_nodeSpacing(nodeSpacing)
{

}

/*!
 * \brief Updates the global origin
 * \param globalOrigin New global origin
 */
void RoadNetwork::setGlobalOrigin(const mace::pose::GeodeticPosition_3D &globalOrigin)
{
    // Track how the internal coordinates are translated to original Cartesian coordinates' origin based on new origin
    mace::pose::CartesianPosition_3D convertedPosition;
    mace::pose::DynamicsAid::GlobalPositionToLocal(m_globalOrigin, globalOrigin, convertedPosition);
    m_iTranslation += std::round(convertedPosition.getXPosition() / m_nodeSpacing);
    m_jTranslation += std::round(convertedPosition.getYPosition() / m_nodeSpacing);
    m_kTranslation += std::round(convertedPosition.getZPosition() / m_nodeSpacing);

    m_globalOrigin = globalOrigin;
}

/*!
 * \brief Adds a node to the road network
 * \param position Node location
 * \param state State of the node. Defaults to RoadNetwork::NodeState::Unexplored
 * \param isOpen Whether the node is open, ie. if the road network could potentially be expanded from this node
 * \return false if a node already exists for this position
 */
bool RoadNetwork::AddNode(const mace::state_space::StatePtr &position, RoadNetwork::NodeType type, bool isOpen)
{
    RoadNetwork_NodeKey key = PositionToNodeKey(position);
    NodeData data;
    data.type = type;
    data.position = position;
    data.isOpen = isOpen;

    return m_graph.AddVertex(key, data);
}

/*!
 * \brief Checks if a node exists at the specified position
 * \param position Location
 * \return Whether the node exists
 */
bool RoadNetwork::NodeExists(const mace::state_space::StatePtr &position)
{
    RoadNetwork_NodeKey key = PositionToNodeKey(position);

    return m_graph.ContainsVertex(key);
}

/*!
 * \brief Adds an edge between nodes
 * \param position1 Position of node 1
 * \param position2 Position of node 2
 * \return Whether the edge was successfully added. Failure indicates either that the edge already exists, or at least
 * one node does not exist
 */
bool RoadNetwork::AddEdge(const mace::state_space::StatePtr &position1, const mace::state_space::StatePtr &position2)
{
    RoadNetwork_NodeKey key1 = PositionToNodeKey(position1);
    RoadNetwork_NodeKey key2 = PositionToNodeKey(position2);

    return m_graph.AddEdge(key1, key2, key1.distanceBetween(key2));
}

/*!
 * \brief Returns whether an edge exists
 * \param position1 Position of node 1
 * \param position2 Position of node 2
 * \return Whether an edge exists
 */
bool RoadNetwork::EdgeExists(const mace::state_space::StatePtr &position1, const mace::state_space::StatePtr &position2)
{
    RoadNetwork_NodeKey key1 = PositionToNodeKey(position1);
    RoadNetwork_NodeKey key2 = PositionToNodeKey(position2);

    return m_graph.ContainsEdge(key1, key2);
}

/*!
 * \brief Returns whether two nodes are connected
 * \param position1 Position of node 1
 * \param position2 Position of node 2
 * \return Whether the nodes are connected
 */
bool RoadNetwork::Connected(const mace::state_space::StatePtr &position1, const mace::state_space::StatePtr &position2)
{
    RoadNetwork_NodeKey key1 = PositionToNodeKey(position1);
    RoadNetwork_NodeKey key2 = PositionToNodeKey(position2);

    return m_graph.Connected(key1, key2);
}

/*!
 * \brief Retrieves adjacent nodes from a specified node
 * \param position Node location
 * \return Adjacent nodes, mapping position to NodeType
 */
std::unordered_map<mace::state_space::StatePtr, RoadNetwork::NodeType> RoadNetwork::GetAdjacentNodes(const mace::state_space::StatePtr &position)
{
    RoadNetwork_NodeKey key = PositionToNodeKey(position);

    auto neighbors = m_graph.GetNeighbors(key);

    std::unordered_map<mace::state_space::StatePtr, RoadNetwork::NodeType> adjacentNodes;

    for (auto keyDataPair : neighbors)
        adjacentNodes.insert({keyDataPair.second.position, keyDataPair.second.type});

    return adjacentNodes;
}

/*!
 * \brief Retrieves nodes connected to a specified node
 * \param position Node location
 * \return Connected nodes, mapping position to NodeType
 */
std::unordered_map<mace::state_space::StatePtr, RoadNetwork::NodeType> RoadNetwork::GetConnectedNodes(const mace::state_space::StatePtr &position)
{
    std::unordered_map<mace::state_space::StatePtr, RoadNetwork::NodeType> results;

    RoadNetwork_NodeKey key = PositionToNodeKey(position);

    auto connected = m_graph.GetConnectedVertices(key);

    for (auto keyDataPair : connected)
        results.insert({keyDataPair.second.position, keyDataPair.second.type});

    return results;
}

/*!
 * \brief Sets the node type
 * \details Typically, this will be used to change the node from RoadNetwork::NodeType::Unexplored to some other state
 * when the node is examined. No effect if the node does not exist
 * \param position Location
 * \param state New type
 */
void RoadNetwork::SetNodeType(const mace::state_space::StatePtr &position, RoadNetwork::NodeType type)
{
    RoadNetwork_NodeKey key = PositionToNodeKey(position);
    NodeData data;
    if (!m_graph.GetVertexData(key, data))
        return;

    data.type = type;
    m_graph.SetVertexData(key, data);
}

/*!
 * \brief Retrieves the type for the node at the specified position
 * \param position Location
 * \param state Node type
 * \return Whether the node exists
 */
bool RoadNetwork::GetNodeType(const mace::state_space::StatePtr &position, RoadNetwork::NodeType &type)
{
    RoadNetwork_NodeKey key = PositionToNodeKey(position);
    NodeData data;
    if (!m_graph.GetVertexData(key, data))
        return false;

    type = data.type;
    return true;
}

/*!
 * \brief Sets whether a node is open
 * \details No effect if the node does not exist
 * \param position Position of node
 * \param isOpen Whether a node is open
 */
void RoadNetwork::SetNodeOpen(const state_space::StatePtr &position, bool isOpen)
{
    RoadNetwork_NodeKey key = PositionToNodeKey(position);
    NodeData data;
    if (!m_graph.GetVertexData(key, data))
        return;

    data.isOpen = isOpen;
    m_graph.SetVertexData(key, data);
}

/*!
 * \brief Retrieves whether the node at the specified position is open
 * \param position Location
 * \param isOpen Whether the node is open
 * \return Whether the node exists
 */
bool RoadNetwork::GetNodeOpen(const state_space::StatePtr &position, bool &isOpen)
{
    RoadNetwork_NodeKey key = PositionToNodeKey(position);
    NodeData data;
    if (!m_graph.GetVertexData(key, data))
        return false;

    isOpen = data.isOpen;
    return true;
}

/*!
 * \brief Performs a breadth-first search to find open nodes
 * \details Results are saved in the order they are found.
 * A breadth-first search will always only search the nodes connected to the start node.
 * \param start Start location
 * \param results A vector of States that contains all open nodes found
 */
void RoadNetwork::FindOpenNodesBFS(const mace::state_space::StatePtr &start,
                                   std::vector<mace::state_space::StatePtr> &results)
{
    results.clear();

    RoadNetwork_NodeKey key = PositionToNodeKey(start);
    graph_t::BFSVisitorType visitor;

    auto lambda = [&](NodeData nodeData)
    {
        if (nodeData.isOpen)
            results.push_back(nodeData.position);
    };

    visitor.AddAction(graph_t::BFSVisitorType::Action::DiscoverVertex, graph_t::GetVisitorActionWrapper(lambda));
    m_graph.Search(visitor, key);
}

/*!
 * \brief Performs a depth-first search to find open nodes
 * \details Results are saved in the order they are found.
 * \param start Start location
 * \param results A vector of States that contains all open nodes found
 * \param connectedOnly Whether to limit the search to the connected portion of the network, or to search the entire network
 */
void RoadNetwork::FindOpenNodesDFS(const mace::state_space::StatePtr &start, std::vector<mace::state_space::StatePtr> &results, bool connectedOnly)
{
    results.clear();

    RoadNetwork_NodeKey key = PositionToNodeKey(start);
    graph_t::DFSVisitorType visitor;

    auto lambda = [&](NodeData nodeData)
    {
        if (nodeData.isOpen)
            results.push_back(nodeData.position);
    };

    visitor.AddAction(graph_t::DFSVisitorType::Action::DiscoverVertex, graph_t::GetVisitorActionWrapper(lambda));
    m_graph.Search(visitor, key, connectedOnly);
}

/*!
 * \brief Initializes an incremental BFS to find unexplored nodes in the graph.
 * \details This function can be used to reduce the search space when searching for unexplored nodes
 * multiple times, as new nodes or edges are added. Only one incremental search can be ongoing at a time.
 */
void RoadNetwork::Init_IncrementalFindOpenNodesBFS()
{
    m_cachedSeenVertices.clear();
    m_cachedResults.clear();
    graph_t::BFSVisitorType visitor;

    auto lambda = [&](NodeData nodeData)
    {
        RoadNetwork_NodeKey key = PositionToNodeKey(nodeData.position);

        if (nodeData.isOpen && m_cachedSeenVertices.find(key) == m_cachedSeenVertices.cend())
        {
            m_cachedResults.push_back(nodeData.position);
            m_cachedSeenVertices.insert(key);
        }
    };

    visitor.AddAction(graph_t::BFSVisitorType::Action::DiscoverVertex, graph_t::GetVisitorActionWrapper(lambda));

    m_graph.InitializeIncrementalSearch(visitor);
}

/*!
 * \brief Initializes an incremental DFS to find unexplored nodes in the graph.
 * \details This function can be used to reduce the search space when searching for unexplored nodes
 * multiple times, as new nodes or edges are added. Only one incremental search can be ongoing at a time.
 * \param connectedOnly Whether the search is limited to a connected subgraph
 */
void RoadNetwork::Init_IncrementalFindOpenNodesDFS(bool connectedOnly)
{
    m_cachedSeenVertices.clear();
    m_cachedResults.clear();
    graph_t::DFSVisitorType visitor;

    auto lambda = [&](NodeData nodeData)
    {
        RoadNetwork_NodeKey key = PositionToNodeKey(nodeData.position);

        if (nodeData.isOpen && m_cachedSeenVertices.find(key) == m_cachedSeenVertices.cend())
        {
            m_cachedResults.push_back(nodeData.position);
            m_cachedSeenVertices.insert(key);
        }
    };

    visitor.AddAction(graph_t::DFSVisitorType::Action::DiscoverVertex, graph_t::GetVisitorActionWrapper(lambda));
    m_graph.InitializeIncrementalSearch(visitor, connectedOnly);
}

/*!
 * \brief Incrementally searches for unexplored nodes
 *
 * \param start Start vertex
 * \param results Results
 */
void RoadNetwork::IncrementalFindOpenNodes(const mace::state_space::StatePtr &start, std::vector<mace::state_space::StatePtr> &results)
{
    RoadNetwork_NodeKey key = PositionToNodeKey(start);
    m_graph.IncrementalSearch(key);

    results = m_cachedResults;
    m_cachedResults.clear();
}

/*!
 * \brief Propogates data throughout the network, using the specified update function
 * \details This function uses a BFS strategy to visit all vertices. The search terminates early if the update function returns true.
 * \param start Start vertex
 * \param updateLambda Update function
 */
void RoadNetwork::PropogateDataBFS(const mace::state_space::StatePtr &start, std::function<bool(RoadNetwork::NodeData &)> updateLambda)
{
    RoadNetwork_NodeKey startKey = PositionToNodeKey(start);

    auto lambda = [&](NodeData nodeData)
    {
        RoadNetwork_NodeKey key = PositionToNodeKey(nodeData.position);
        if (updateLambda(nodeData))
            throw EarlyTermination();
        m_graph.SetVertexData(key, nodeData);
    };

    graph_t::BFSVisitorType visitor;
    visitor.AddAction(graph_t::BFSVisitorType::Action::DiscoverVertex, graph_t::GetVisitorActionWrapper(lambda));

    try
    {
        m_graph.Search(visitor, startKey);
    } catch (EarlyTermination e) {}
}
/*!
 * \brief Propogates data throughout the network, using the specified update function
 * \details This function uses a DFS strategy to visit all vertices. The search terminates early if the update function returns true.
 * \param start Start vertex
 * \param updateLambda Update function
 * \param Whether to limit the search to a connected subgraph
 */
void RoadNetwork::PropogateDataDFS(const mace::state_space::StatePtr &start, std::function<bool(RoadNetwork::NodeData &)> updateLambda, bool connectedOnly)
{
    RoadNetwork_NodeKey key = PositionToNodeKey(start);

    auto lambda = [&](NodeData nodeData)
    {
        RoadNetwork_NodeKey key = PositionToNodeKey(nodeData.position);
        updateLambda(nodeData);
        m_graph.SetVertexData(key, nodeData);
    };

    graph_t::DFSVisitorType visitor;
    visitor.AddAction(graph_t::DFSVisitorType::Action::DiscoverVertex, graph_t::GetVisitorActionWrapper(lambda));

    try
    {
        m_graph.Search(visitor, key, connectedOnly);
    } catch (EarlyTermination e) {}
}

/*!
 * \brief Generates a task to go to a node
 * \details This presently generates a loiter task with a duration of 0 seconds. The caller
 * is responsible for setting the timestamp, and required start/end parameters.
 * \param position Position of the node
 * \param creatorID Creator ID
 * \param taskID task ID
 * \return The generated task, or a nullptr if the position does not correspond to a node in the network
 */
TaskDescriptorPtr RoadNetwork::GenerateGotoNodeTask(const mace::state_space::StatePtr &position,  uint64_t creatorID, uint8_t taskID)
{
    TaskDescriptorPtr task = nullptr;

    if (NodeExists(position))
    {
        mace::pose::CartesianPosition_3D *pos3DC = dynamic_cast<mace::pose::CartesianPosition_3D *>(position.get());
        mace::pose::CartesianPosition_2D *pos2DC = dynamic_cast<mace::pose::CartesianPosition_2D *>(position.get());
        mace::pose::GeodeticPosition_3D *pos3DG = dynamic_cast<mace::pose::GeodeticPosition_3D *>(position.get());
        mace::pose::GeodeticPosition_2D *pos2DG = dynamic_cast<mace::pose::GeodeticPosition_2D *>(position.get());

        if (pos3DC)
        {
            auto loiterTask = std::make_shared<TaskLoiterDescriptor<mace::pose::CartesianPosition_3D>>(creatorID, taskID);
            loiterTask->setLoiterPosition(*pos3DC);
            loiterTask->setDuration(0);
            task = loiterTask;
        }
        else if (pos2DC)
        {
            auto loiterTask = std::make_shared<TaskLoiterDescriptor<mace::pose::CartesianPosition_2D>>(creatorID, taskID);
            loiterTask->setLoiterPosition(*pos2DC);
            loiterTask->setDuration(0);
            task = loiterTask;
        }
        else if (pos3DG)
        {
            auto loiterTask = std::make_shared<TaskLoiterDescriptor<mace::pose::GeodeticPosition_3D>>(creatorID, taskID);
            loiterTask->setLoiterPosition(*pos3DG);
            loiterTask->setDuration(0);
            task = loiterTask;
        }
        else if (pos2DG)
        {
            auto loiterTask = std::make_shared<TaskLoiterDescriptor<mace::pose::GeodeticPosition_2D>>(creatorID, taskID);
            loiterTask->setLoiterPosition(*pos2DG);
            loiterTask->setDuration(0);
            task = loiterTask;
        }
        else
        {
            throw std::runtime_error("Unsupported position type when generating a goto node task");
        }
    }

    return task;
}

/*!
 * \brief Converts real coordinates to the coordinates used internally as a node key
 * \param position Location
 * \return Computed node key
 */
RoadNetwork_NodeKey RoadNetwork::PositionToNodeKey(const mace::state_space::StatePtr &position)
{
    RoadNetwork_NodeKey key;

    mace::pose::CartesianPosition_3D *pos3DC = dynamic_cast<mace::pose::CartesianPosition_3D *>(position.get());
    if (pos3DC)
    {
        key.i = std::round(pos3DC->getXPosition() / m_nodeSpacing) + m_iTranslation;
        key.j = std::round(pos3DC->getYPosition() / m_nodeSpacing) + m_jTranslation;
        key.k = std::round(pos3DC->getZPosition() / m_nodeSpacing) + m_kTranslation;

        return key;
    }

    mace::pose::CartesianPosition_2D *pos2DC = dynamic_cast<mace::pose::CartesianPosition_2D *>(position.get());
    if (pos2DC)
    {
        key.i = std::round(pos2DC->getXPosition() / m_nodeSpacing) + m_iTranslation;
        key.j = std::round(pos2DC->getYPosition() / m_nodeSpacing) + m_jTranslation;
        key.k = 0 + m_kTranslation;

        return key;
    }

    mace::pose::GeodeticPosition_3D *pos3DG = dynamic_cast<mace::pose::GeodeticPosition_3D *>(position.get());
    if (pos3DG)
    {
        mace::pose::CartesianPosition_3D convertedPosition;
        mace::pose::DynamicsAid::GlobalPositionToLocal(m_globalOrigin, *pos3DG, convertedPosition);
        key.i = std::round(convertedPosition.getXPosition() / m_nodeSpacing) + m_iTranslation;
        key.j = std::round(convertedPosition.getYPosition() / m_nodeSpacing) + m_jTranslation;
        key.k = std::round(convertedPosition.getZPosition() / m_nodeSpacing) + m_kTranslation;

        return key;
    }

    mace::pose::GeodeticPosition_2D *pos2DG = dynamic_cast<mace::pose::GeodeticPosition_2D *>(position.get());
    if (pos2DG)
    {
        mace::pose::GeodeticPosition_3D convertedGlobalPosition;
        mace::pose::CartesianPosition_3D convertedLocalPosition;
        convertedGlobalPosition.setAltitude(0.0);
        convertedGlobalPosition.setLatitude(pos2DG->getLatitude());
        convertedGlobalPosition.setLongitude(pos2DG->getLongitude());

        mace::pose::DynamicsAid::GlobalPositionToLocal(m_globalOrigin, convertedGlobalPosition, convertedLocalPosition);
        key.i = std::round(convertedLocalPosition.getXPosition() / m_nodeSpacing) + m_iTranslation;
        key.j = std::round(convertedLocalPosition.getYPosition() / m_nodeSpacing) + m_jTranslation;
        key.k = std::round(convertedLocalPosition.getZPosition() / m_nodeSpacing) + m_kTranslation;

        return key;
    }

    throw std::runtime_error("Invalid position type");
}

/**************** NodeKey ****************/

/*!
 * \brief Calculates the distance between this and another road network node, in the scaled coordinate system
 * \param other Other node
 * \return Distance
 */
double RoadNetwork_NodeKey::distanceBetween(const RoadNetwork_NodeKey &other)
{

    return sqrt(((double) (other.i - i) * (other.i - i))
              + ((double) (other.j - j) * (other.j - j))
              + ((double) (other.k - k) * (other.k - k)));
}

/*!
 * \brief Equality operator
 */
bool RoadNetwork_NodeKey::operator==(const RoadNetwork_NodeKey &other) const
{
    return i == other.i && j == other.j && k == other.k;
}

/*!
 * \brief Inequality operator
 */
bool RoadNetwork_NodeKey::operator!=(const RoadNetwork_NodeKey &other) const
{
    return !(*this == other);
}

}
}
