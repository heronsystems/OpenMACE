#ifndef ROAD_NETWORK_H
#define ROAD_NETWORK_H

#include "graph.h"
#include "base/state_space/state.h"

#include "base/pose/cartesian_position_2D.h"
#include "base/pose/cartesian_position_3D.h"
#include "base/pose/geodetic_position_2D.h"
#include "base/pose/geodetic_position_3D.h"
#include "base/pose/dynamics_aid.h"

#include "data_tasks/task_descriptor.h"

#include <vector>

#include <functional>

namespace mace {
namespace graphs {

/*!
 * \brief The RoadNetwork_NodeKey class is used internally by RoadNetwork to save uniquely identify nodes
 * \details Real 3D Cartesian positions are mapped to an integer coordinate system based on a spacing parameter
 * set when the RoadNetwork is constructed. Given coordinates (x, y, z) and spacing parameter s, the corresponding
 * coordinates are (i = round(x / s), j = round(y / s), k = round(z / s)). Every possible node specified by this struct
 * represents a cube with width s centered on the coordinates (i * s, j * s, k * s); consequently all real coordinates
 * within this cube are considered equal by the RoadNetwork class.
 *
 * Note: This struct is passed by value in many calls by the underlying graph library.
 */
typedef struct RoadNetwork_NodeKey
{
    friend class RoadNetwork;

    int i;
    int j;
    int k;

    double distanceBetween(const RoadNetwork_NodeKey &other);

    bool operator==(const RoadNetwork_NodeKey &other) const;

    bool operator!=(const RoadNetwork_NodeKey &other) const;
} RoadNetwork_NodeKey;

}
}

namespace std
{
template<>
struct hash<mace::graphs::RoadNetwork_NodeKey>
{
    size_t operator()(const mace::graphs::RoadNetwork_NodeKey &key) const
    {
        size_t h1, h2, h3;
        h1 = std::hash<double>{}(key.i);
        h2 = std::hash<double>{}(key.j);
        h3 = std::hash<double>{}(key.k);

        return (h1 ^ (h2 << 1)) ^ (h3 << 1);
    }
};
}

namespace mace {
namespace graphs {

/*!
 * \brief The RoadNetwork class represents a road network
 */
class RoadNetwork
{
public:
    /*!
     * \brief The NodeState enum represents the possible node types in the graph
     */
    typedef enum class NodeType
    {
        Unexplored,
        IsRoad,
        FreeSpace,
        Obstacle
    } NodeType;

    /*!
     * \brief The NodeData struct saves information about a node
     * \details Note: This struct is passed by value in many calls by the underlying graph library.
     */
    typedef struct NodeData
    {
        mace::state_space::StatePtr position;
        NodeType type;
        bool isOpen;
    } NodeData;

    RoadNetwork(const mace::pose::GeodeticPosition_3D &globalOrigin, double nodeSpacing = 1.0);

    void setGlobalOrigin(const mace::pose::GeodeticPosition_3D &globalOrigin);

    bool AddNode(const mace::state_space::StatePtr &position, NodeType type = NodeType::Unexplored, bool isOpen = true);
    bool NodeExists(const mace::state_space::StatePtr &position);

    bool AddEdge(const mace::state_space::StatePtr &position1, const mace::state_space::StatePtr &position2);
    bool EdgeExists(const mace::state_space::StatePtr &position1, const mace::state_space::StatePtr &position2);

    bool Connected(const mace::state_space::StatePtr &position1, const mace::state_space::StatePtr &position2);

    std::unordered_map<mace::state_space::StatePtr, RoadNetwork::NodeType> GetAdjacentNodes(const mace::state_space::StatePtr &position);
    std::unordered_map<mace::state_space::StatePtr, RoadNetwork::NodeType> GetConnectedNodes(const mace::state_space::StatePtr &position);

    void SetNodeType(const mace::state_space::StatePtr &position, NodeType type);
    bool GetNodeType(const mace::state_space::StatePtr &position, NodeType &type);

    void SetNodeOpen(const mace::state_space::StatePtr &position, bool isOpen);
    bool GetNodeOpen(const mace::state_space::StatePtr &position, bool &isOpen);

    void FindOpenNodesBFS(const mace::state_space::StatePtr &start,
                          std::vector<mace::state_space::StatePtr> &results);

    void FindOpenNodesDFS(const mace::state_space::StatePtr &start,
                          std::vector<mace::state_space::StatePtr> &results,
                          bool connectedOnly = true);

    void Init_IncrementalFindOpenNodesBFS();
    void Init_IncrementalFindOpenNodesDFS(bool connectedOnly = true);

    void IncrementalFindOpenNodes(const mace::state_space::StatePtr &start, std::vector<mace::state_space::StatePtr> &results);

    void PropogateDataBFS(const mace::state_space::StatePtr &start, std::function<bool(NodeData &)> updateLambda);
    void PropogateDataDFS(const mace::state_space::StatePtr &start, std::function<bool(NodeData &)> updateLambda, bool connectedOnly = true);

    TaskDescriptorPtr GenerateGotoNodeTask(const mace::state_space::StatePtr &position, uint64_t creatorID, uint8_t taskID);

private:
    /*!
     * \brief The EarlyTermination class is used to terminate a search early.
     */
    class EarlyTermination : std::exception
    {
    public:
        EarlyTermination()
        {
        }
    };

    using graph_t = Graph<Undirected, RoadNetwork_NodeKey, NodeData>;

    graph_t m_graph;

    mace::pose::GeodeticPosition_3D m_globalOrigin;
    double m_nodeSpacing;

    // Translations used if the global origin is changed from what it is set as in the constructor.
    // The internal integer coordinates would be invalidated, but updating every node is costly, so
    // we track what the internal coordinates would be with the originally set global origin.
    double m_iTranslation = 0;
    double m_jTranslation = 0;
    double m_kTranslation = 0;

    std::unordered_set<RoadNetwork_NodeKey> m_cachedSeenVertices;
    std::vector<mace::state_space::StatePtr> m_cachedResults;

    RoadNetwork_NodeKey PositionToNodeKey(const mace::state_space::StatePtr &position);
};

}
}
#endif // ROAD_NETWORK_H
