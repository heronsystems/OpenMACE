#ifndef GRAPH_H
#define GRAPH_H

#include <map>
#include <unordered_map>
#include <unordered_set>

#include <boost/lockfree/queue.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/depth_first_search.hpp>

#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/incremental_components.hpp>

#include "visitors/bfs_visitor.h"
#include "visitors/dfs_visitor.h"


#include <functional>

namespace mace {
namespace graphs {


using Directed = boost::directedS;
using Undirected =  boost::undirectedS;


/*!
 * \brief The Graph class is a wrapper around the Boost Graph Library.
 * \details Note: Vertex, Edge, Graph data are all passed by value.
 *
 * This class is not thread safe.
 *
 * \tparam Direction Whether the graph is directed or undirected. Set to Directed or Undirected
 * \tparam VertexKey Key uniquely identifying a vertex. This class must provide an equality operator, and a std::hash specialization
 * \tparam VertexData Data saved at each vertex
 * \tparam EdgeData Data saved at each edge. Two VertexKeys act as the key for an edge
 * \tparam GraphData Data saved for the entire graph
 */
template <typename Direction,
          typename VertexKey,
          typename VertexData,
          typename EdgeData = boost::no_property,
          typename GraphData = boost::no_property>
class Graph
{
    static_assert(std::is_base_of<Directed, Direction>() || std::is_base_of<Undirected, Direction>(), "Must be a directed or undirected graph");

    /*!
     * \brief The EarlyTermination class is used internally by the Graph class to terminate a depth-first search once the connected portion
     * of the graph is searched, from the starting point.
     *
     * \details By default, the BGL will search the full graph when using depth-first search, but is guaranteed to search vertices connected
     * to the start vertex first. Modifying the discover vertex lambda of a user's DFS visitor to check connectivity allows the search to be
     * terminated by throwing this exception after the connected portion of the graph has been fully searched.
     *
     * This is not necessary for breadth-first search, which will always search the connected portion of the graph.
     */
    class EarlyTermination : public std::exception
    {
    public:
        EarlyTermination()
        {
        }
    };

    /*!
     * \brief The Vertex struct stores data related to each vertex.
     */
    typedef struct Vertex
    {
        VertexKey key;
        VertexData data;
    } Vertex;

    /*!
     * \brief The Edge struct stores data related to each edge.
     */
    typedef struct Edge
    {
        EdgeData data;
        float weight;
    } Edge;


    // hash_setS for OutEdgeListS parameter disallows parallel edges
    using AdjacencyList = boost::adjacency_list<boost::hash_setS, boost::vecS, Direction, Vertex, Edge, GraphData>;
    using VertexIndex = typename boost::graph_traits<AdjacencyList>::vertex_descriptor;
    using VertexIterator = typename boost::graph_traits<AdjacencyList>::vertex_iterator;
    using EdgeIndex = typename boost::graph_traits<AdjacencyList>::edge_descriptor;
    using EdgeIterator = typename boost::graph_traits<AdjacencyList>::edge_iterator;

    // Types used by disjoint sets
    using Rank = std::unordered_map<VertexIndex, std::size_t>;
    using Parent = std::unordered_map<VertexIndex, VertexIndex>;
    using RankPMap = boost::associative_property_map<Rank>;
    using ParentPMap = boost::associative_property_map<Rank>;

    // Cached searches
#ifdef _WIN32
    using IndexMap = boost::vec_adj_list_vertex_id_map<Graph<Direction, VertexKey, VertexData, EdgeData, GraphData>::Vertex, unsigned int>;
#else
    using IndexMap = boost::vec_adj_list_vertex_id_map<Graph<Direction, VertexKey, VertexData, EdgeData, GraphData>::Vertex, unsigned long int>;
#endif

    using ColorMap = boost::vector_property_map<boost::default_color_type, IndexMap>;

public:
    // Defines the appropriate visitors for use in BFS/DFS
    using BFSVisitorType =  BFSVisitor<AdjacencyList, VertexIndex, EdgeIndex>;
    using DFSVisitorType = DFSVisitor<AdjacencyList, VertexIndex, EdgeIndex>;

    /*!
     * \brief Returns a wrapper function that translates around a user lambda to the type needed by a visitor acting on vertices
     * \param lambda Lambda
     * \return Wrapper function that translates around a user lambda to the type needed by a visitor acting on vertices
     */
    static std::function<void(VertexIndex, AdjacencyList)> GetVisitorActionWrapper(std::function<void(VertexData)> lambda)
    {
        auto wrapper = [lambda](VertexIndex v, AdjacencyList g)
        {
            lambda(g[v].data);
        };

        return wrapper;
    }

    /*!
     * \brief Returns a wrapper function that translates around a user lambda to the type needed by a visitor acting on vertices
     * \overload
     * \details This function is disabled if a GraphData type has not been specified
     * \param lambda Lambda
     * \return Wrapper function that translates around a user lambda to the type needed by a visitor acting on vertices
     */
    template<typename G = boost::no_property>
    static typename std::enable_if<!std::is_same<G, GraphData>::value, std::function<void(VertexIndex, AdjacencyList)>>::type GetVisitorActionWrapper(std::function<void(VertexData, GraphData)> lambda)
    {
        auto wrapper = [lambda](VertexIndex v, AdjacencyList g)
        {
            lambda(g[v].data, g[boost::graph_bundle].data);
        };

        return wrapper;
    }

    /*!
     * \brief Returns a wrapper function that translates around a user lambda to the type needed by a visitor acting on edges
     * \overload
     * \details This function is disabled if an EdgeData type has not been specified
     * \param lambda Lambda
     * \return Wrapper function that translates around a user lambda to the type needed by a visitor acting on edges
     */
    template<typename E = boost::no_property>
    static typename std::enable_if<!std::is_same<E, EdgeData>::value, std::function<void(EdgeIndex, AdjacencyList)>>::type GetVisitorActionWrapper(std::function<void(EdgeData)> lambda)
    {
        auto wrapperLambda = [lambda](EdgeIndex e, AdjacencyList g)
        {
            lambda(g[e].data);
        };

        return wrapperLambda;
    }

    /*!
     * \brief Returns a wrapper function that translates around a user lambda to the type needed by a visitor acting on edges
     * \overload
     * \details This function is disabled if ether an EdgeData or GraphData type has not been specified
     * \param lambda Lambda
     * \return Wrapper function that translates around a user lambda to the type needed by a visitor acting on edges
     */
    template<typename E = boost::no_property, typename G = boost::no_property>
    static typename std::enable_if<!std::is_same<E, EdgeData>::value && !std::is_same<G, GraphData>::value, std::function<void(EdgeIndex, AdjacencyList)>>::type  GetVisitorActionWrapper(std::function<void(EdgeData, GraphData)> lambda)
    {
        auto wrapperLambda = [lambda](EdgeIndex e, AdjacencyList g)
        {
            lambda(g[e].data, g[boost::graph_bundle].data);
        };

        return wrapperLambda;
    }

    /*!
     * \brief Constructor
     */
    Graph() :
        m_rankPMap(m_rank),
        m_parentPMap(m_parent),
        m_disjointSets(m_rankPMap, m_parentPMap)
    {
    }

    /*!
     * \brief Adds a vertex with corresponding data
     * \param key Unique identifier
     * \param data Data
     * \return false if the vertex was previously added
     */
    bool AddVertex(VertexKey key, VertexData data)
    {
        if (ContainsVertex(key))
            return false;

        Vertex vertex;
        vertex.key = key;
        vertex.data = data;
        VertexIndex v = boost::add_vertex(vertex, m_adjacencyList);
        m_vertexIndices.insert({key, v});

        m_disjointSets.make_set(v);

        // Ensures that incremental searches know of the new vertex
        if (m_incrementalSearchDataBFS)
        {
            boost::put(m_incrementalSearchDataBFS->colorMap, v, boost::default_color_type::white_color);
            m_incrementalSearchDataBFS->newVertices.insert(key);
        }
        else if (m_incrementalSearchDataDFS)
        {
            boost::put(m_incrementalSearchDataDFS->colorMap, v, boost::default_color_type::white_color);
            m_incrementalSearchDataDFS->newVertices.insert(key);
        }

        return true;
    }

    /*!
     * \brief Returns whether the graph contains the specified vertex
     * \param key Unique identifier
     * \return Whether the graph contains the specified vertex
     */
    bool ContainsVertex(VertexKey key)
    {
        return m_vertexIndices.find(key) != m_vertexIndices.cend();
    }

    /*!
     * \brief Retreieves the data at the specified vertex
     * \param key Unique identifier
     * \param data Updated to reflect the saved data
     * \return Whether the graph contains the specified vertex
     */
    bool GetVertexData(VertexKey key, VertexData &data)
    {
        if (!ContainsVertex(key))
            return false;

        VertexIndex v = m_vertexIndices.at(key);
        data = m_adjacencyList[v].data;
        return true;
    }

    /*!
     * \brief Sets the data at the specified vertex. No effect if the vertex does not exist.
     * \param key Unique identifier
     * \param data Data
     */
    void SetVertexData(VertexKey key, VertexData data)
    {
        if (!ContainsVertex(key))
            return;

        VertexIndex v = m_vertexIndices.at(key);
        m_adjacencyList[v].data = data;
    }

    /*!
     * \brief Adds an edge to the graph
     * \details The vertex keys keyU and keyV uniquely identify the edge. If the graph is directed, the edge direction is keyU -> keyV,
     * and (keyU, keyV) is different than (keyV, keyU).
     * \param keyU Unique identifier for vertex U
     * \param keyV Unique identifier for vertex V
     * \param weight Edge weight
     * \param data Data corresponding the edge
     * \return Whether the edge was successfully added. Failure indicates either at least one vertex not existing, or that the edge
     * already existed
     */
    bool AddEdge(VertexKey keyU, VertexKey keyV, float weight = 1, EdgeData data = EdgeData())
    {
        Edge edge;
        edge.data = data;
        edge.weight = weight;
        VertexIndex u, v;

        auto vertexIt = m_vertexIndices.find(keyU);
        if (vertexIt == m_vertexIndices.cend())
            return false;
        u = vertexIt->second;

        auto vertexIt2 = m_vertexIndices.find(keyV);
        if (vertexIt2 == m_vertexIndices.cend())
            return false;
        v = vertexIt2->second;

        auto edgeAddedPair = boost::add_edge(u, v, edge, m_adjacencyList);
        if (edgeAddedPair.second)
        {
            m_disjointSets.union_set(u, v);

            // We need to ensure that vertex u is colored white for incremental searches, and marked as a new vertex,
            // in order to ensure that the new edge is examined.
            // Vertex u must be used as in directed graphs, the edge is from u to v.
            if (m_incrementalSearchDataBFS)
            {
                if (boost::get(m_incrementalSearchDataBFS->colorMap, u) != boost::default_color_type::white_color
                        || (std::is_same<Undirected, Direction>()
                            && boost::get(m_incrementalSearchDataBFS->colorMap, v) != boost::default_color_type::white_color))
                {
                    boost::put(m_incrementalSearchDataBFS->colorMap, u, boost::default_color_type::white_color);

                    m_incrementalSearchDataBFS->fakeNewVertices.insert(keyU);
                    m_incrementalSearchDataBFS->newEdges.insert({keyU, keyV});
                } // Else the edge would be examined without needing to mark it as a new edge, or a vertex as being unexamined (fake new)
            }
            else if (m_incrementalSearchDataDFS)
            {
                if (boost::get(m_incrementalSearchDataDFS->colorMap, u) != boost::default_color_type::white_color
                        || (std::is_same<Undirected, Direction>()
                            && boost::get(m_incrementalSearchDataDFS->colorMap, v) != boost::default_color_type::white_color))
                {
                    boost::put(m_incrementalSearchDataDFS->colorMap, u, boost::default_color_type::white_color);

                    m_incrementalSearchDataDFS->fakeNewVertices.insert(keyU);
                    m_incrementalSearchDataDFS->newEdges.insert({keyU, keyV});
                } // Else the edge would be examined without needing to mark it as a new edge, or a vertex as being unexamined (fake new)
            }

            return true;
        }
        return false;
    }

    /*!
     * \brief Returns whether the graph contains the specified edge
     * \param keyU Unique identifier for vertex U
     * \param keyV Unique identifier for vertex V
     * \return Whether the edge exists
     */
    bool ContainsEdge(VertexKey keyU, VertexKey keyV)
    {
        if (!ContainsVertex(keyU))
            return false;

        if (!ContainsVertex(keyV))
            return false;

        VertexIndex u, v;

        u = m_vertexIndices.find(keyU)->second;
        v = m_vertexIndices.find(keyV)->second;

        return boost::edge(u, v, m_adjacencyList).second;
    }

    /*!
     * \brief Retrieves the neighbors of a vertex
     * \param key Vertex key
     * \return All neighbors of the vertex, mapping key to data
     */
    std::unordered_map<VertexKey, VertexData> GetNeighbors(VertexKey key)
    {
        std::unordered_map<VertexKey, VertexData> neighbors;
        if (ContainsVertex(key))
        {
            VertexIndex v = m_vertexIndices.at(key);
            auto beginEndPair = boost::adjacent_vertices(v, m_adjacencyList);
            auto it = beginEndPair.first;
            while (it != beginEndPair.second)
            {
                auto &vertex = m_adjacencyList[*it];
                neighbors.insert({vertex.key, vertex.data});
                ++it;
            }
        }

        return neighbors;
    }

    /*!
     * \brief Retrieves the edge data for the specified edge
     * \details This function is disabled if the user has not specified an EdgeData type
     * \param keyU Unique identifier for vertex U
     * \param keyV Unique identifier for vertex V
     * \param data Set to the saved edge data
     * \return Whether the edge exists
     */
    template<typename E = boost::no_property>
    typename std::enable_if<!std::is_same<E, EdgeData>::value, bool>::type GetEdgeData(VertexKey keyU, VertexKey keyV, EdgeData &data)
    {
        if (!ContainsVertex(keyU))
            return false;

        if (!ContainsVertex(keyV))
            return false;

        VertexIndex u, v;

        u = m_vertexIndices.find(keyU)->second;
        v = m_vertexIndices.find(keyV)->second;

        auto edgeDataExistsPair = boost::edge(u, v, m_adjacencyList);
        data = m_adjacencyList[edgeDataExistsPair.first].data;
        return edgeDataExistsPair.second;
    }

    /*!
     * \brief Sets the edge data for the specified edge
     * \details This function is disabled if the EdgeData type is not specified. No effect if the edge does not exist
     * \param keyU Unique identifier for vertex U
     * \param keyV Unique identifier for vertex V
     * \param data Edge data
     */
    template<typename E = boost::no_property>
    typename std::enable_if<!std::is_same<E, EdgeData>::value>::type SetEdgeData(VertexKey keyU, VertexKey keyV, EdgeData data)
    {
        if (!ContainsEdge(keyU, keyV))
            return;

        VertexIndex u, v;

        u = m_vertexIndices.find(keyU)->second;
        v = m_vertexIndices.find(keyV)->second;

        auto edgeDataExistsPair = boost::edge(u, v, m_adjacencyList);
        m_adjacencyList[edgeDataExistsPair.first].data = data;
    }

    /*!
     * \brief Retrieves the edge weight
     * \param keyU Unique identifier for vertex U
     * \param keyV Unique identifier for vertex V
     * \param weight Edge weight
     * \return Whether the edge exists
     */
    bool GetEdgeWeight(VertexKey keyU, VertexKey keyV, float &weight)
    {
        if (!ContainsEdge(keyU, keyV))
            return false;

        VertexIndex u, v;

        u = m_vertexIndices.find(keyU)->second;
        v = m_vertexIndices.find(keyV)->second;

        auto edgeDataExistsPair = boost::edge(u, v, m_adjacencyList);
        weight = m_adjacencyList[edgeDataExistsPair.first].weight;

        return true;
    }

    /*!
     * \brief Sets the edge weight
     * \details This function has no effect if the edge does not exist
     * \param keyU Unique identifier for vertex U
     * \param keyV Unique identifier for vertex V
     * \param weight Edge weight
     */
    void SetEdgeWeight(VertexKey keyU, VertexKey keyV, float weight)
    {
        if (!ContainsEdge(keyU, keyV))
            return;

        VertexIndex u, v;

        u = m_vertexIndices.find(keyU)->second;
        v = m_vertexIndices.find(keyV)->second;

        auto edgeDataExistsPair = boost::edge(u, v, m_adjacencyList);
        m_adjacencyList[edgeDataExistsPair.first].weight = weight;
    }

    /*!
     * \brief Sets global graph data
     * \details This function is disabled if a GraphData type is not specified
     */
    template<typename G = boost::no_property>
    typename std::enable_if<!std::is_same<G, GraphData>::value>::type SetGraphData(GraphData data)
    {
        m_adjacencyList[boost::graph_bundle] = data;
    }

    /*!
     * \brief Retrieves global graph data
     * \details This function is disabled if a GraphData type is not specified
     * \return Global graph data
     */
    template<typename G = boost::no_property>
    typename std::enable_if<!std::is_same<G, GraphData>::value, GraphData>::type GetGraphData()
    {
        return m_adjacencyList[boost::graph_bundle];
    }

    /*!
     * \brief Returns whether the specified vertices are connected
     * \param keyU Unique identifier for vertex U
     * \param keyV Unique identifier for vertex V
     * \return Whether the specified vertices are connected
     */
    bool Connected(VertexKey keyU, VertexKey keyV)
    {
        if (!ContainsVertex(keyU))
            return false;

        if (!ContainsVertex(keyV))
            return false;

        VertexIndex u, v;

        u = m_vertexIndices.find(keyU)->second;
        v = m_vertexIndices.find(keyV)->second;

        return m_disjointSets.find_set(u) == m_disjointSets.find_set(v);
    }

    /*!
     * \brief Performs a breadth-first search from the start vertex
     * \details The search will only take place over those vertices connected to the start vertex.
     * The visitor will call any specified lambdas at predefined points in the execution of the search.
     * To terminate the search early, throw and catch an exception in one of these lambdas when the appropriate
     * termination criteria is met.
     * \param visitor Visitor object that has functions invoked at predefined points during the search
     * \param start Start vertex
     */
    void Search(BFSVisitorType visitor, VertexKey start)
    {
        if (!ContainsVertex(start))
            return;

        boost::queue<VertexIndex> buffer;
        VertexIndex s = m_vertexIndices.find(start)->second;

        auto indexMap = boost::get(boost::vertex_index, m_adjacencyList);
        auto colorMap = boost::make_vector_property_map<boost::default_color_type>(indexMap);

        boost::breadth_first_search(m_adjacencyList, s, buffer, visitor, colorMap);
    }

    /*!
     * \brief Performs a depth-first search from the start vertex
     * \overload
     * \details The search will only take place over those vertices connected to the start vertex if connectedOnly is set to true.
     * The visitor will call any specified lambdas at predefined points in the execution of the search.
     * To terminate the search early, throw and catch an exception in one of these lambdas when the appropriate
     * termination criteria is met.
     * \param visitor Visitor object that has functions invoked at predefined points during the search
     * \param start Start vertex
     * \param connectedOnly Whether only the the vertices connected to the start vertex should be searched, or the entire graph
     */
    void Search(DFSVisitorType visitor, VertexKey start, bool connectedOnly = true)
    {
        if (!ContainsVertex(start))
            return;

        boost::queue<VertexIndex> buffer;
        VertexIndex s = m_vertexIndices.find(start)->second;


        auto indexMap = boost::get(boost::vertex_index, m_adjacencyList);
        auto colorMap = boost::make_vector_property_map<boost::default_color_type>(indexMap);

        if (connectedOnly)
        {
            // Adds a lambda that terminates if a disconnected vertex is examined
            auto discoverLambda = visitor.GetVertexAction(DFSVisitorType::Action::DiscoverVertex);
            if (discoverLambda != nullptr)
            {
                visitor.AddAction(DFSVisitorType::Action::DiscoverVertex, [&](VertexIndex v, AdjacencyList g)
                {
                    if (m_disjointSets.find_set(s) != m_disjointSets.find_set(v))
                        throw EarlyTermination();

                    discoverLambda(v, g);
                });
            }
            else
            {
                visitor.AddAction(DFSVisitorType::Action::DiscoverVertex, [&](VertexIndex v, AdjacencyList g)
                {
                    UNUSED(g);
                    if (m_disjointSets.find_set(s) != m_disjointSets.find_set(v))
                        throw EarlyTermination();
                });
            }

            try
            {
                boost::depth_first_search(m_adjacencyList, visitor, colorMap, s);
            }
            catch (EarlyTermination e)
            {
            }

            return;
        }

        boost::depth_first_search(m_adjacencyList, visitor, colorMap, s);
    }

    /*!
     * \brief Ends an incremental search
     */
    void EndIncrementalSearch()
    {
        m_incrementalSearchDataBFS.reset();
        m_incrementalSearchDataDFS.reset();
    }

    /*!
     * \brief Initializes an incremental breadth-first seearch
     * \details Only one incremental search of any type can run at a time
     * \param visitor Visitor
     */
    void InitializeIncrementalSearch(BFSVisitorType visitor)
    {
        EndIncrementalSearch();

        m_incrementalSearchDataBFS.reset(new SearchCache_BFS());
        m_incrementalSearchDataBFS->visitor = InitializeIncrementalVisitor(visitor);
        m_incrementalSearchDataBFS->indexMap = boost::get(boost::vertex_index, m_adjacencyList);
        m_incrementalSearchDataBFS->colorMap = boost::make_vector_property_map<boost::default_color_type>(m_incrementalSearchDataBFS->indexMap);

        InitializeColorMap(m_incrementalSearchDataBFS->colorMap);
    }

    /*!
     * \brief Initializes an incremental depth-first seearch
     * \details Only one incremental search of any type can run at a time
     * \param visitor Visitor
     */
    void InitializeIncrementalSearch(DFSVisitorType visitor, bool connectedOnly = true)
    {
        EndIncrementalSearch();

        m_incrementalSearchDataDFS.reset(new SearchCache_DFS());
        m_incrementalSearchDataDFS->visitor = InitializeIncrementalVisitor(visitor);
        m_incrementalSearchDataDFS->indexMap = boost::get(boost::vertex_index, m_adjacencyList);
        m_incrementalSearchDataDFS->colorMap = boost::make_vector_property_map<boost::default_color_type>(m_incrementalSearchDataBFS->indexMap);
        m_incrementalSearchDataDFS->connectedOnly = connectedOnly;

        InitializeColorMap(m_incrementalSearchDataDFS->colorMap);

        if (connectedOnly)
        {
            // Adds a lambda that terminates if a disconnected vertex is examined
            auto discoverLambda = visitor.GetVertexAction(DFSVisitorType::Action::DiscoverVertex);
            if (discoverLambda != nullptr)
            {
                visitor.AddAction(DFSVisitorType::Action::DiscoverVertex, [&](VertexIndex v, AdjacencyList g)
                {
                    if (m_disjointSets.find_set(m_incrementalSearchDataDFS->start) != m_disjointSets.find_set(v))
                        throw EarlyTermination();

                    discoverLambda(v, g);
                });
            }
            else
            {
                visitor.AddAction(DFSVisitorType::Action::DiscoverVertex, [&](VertexIndex v, AdjacencyList g)
                {
                    UNUSED(g);
                    if (m_disjointSets.find_set(m_incrementalSearchDataDFS->start) != m_disjointSets.find_set(v))
                        throw EarlyTermination();
                });
            }
        }
    }

    /*!
     * \brief Runs an incremental search
     * \param start Start vertex
     */
    void IncrementalSearch(VertexKey start)
    {
        if (!ContainsVertex(start))
            return;

        VertexIndex s = m_vertexIndices.find(start)->second;

        if (m_incrementalSearchDataBFS != nullptr)
        {
            boost::breadth_first_visit(m_adjacencyList, s,
                                       m_incrementalSearchDataBFS->buffer,
                                       m_incrementalSearchDataBFS->visitor,
                                       m_incrementalSearchDataBFS->colorMap);

            for (VertexKey key: m_incrementalSearchDataBFS->newVertices)
            {
                VertexIndex u = m_vertexIndices.find(key)->second;

                if (Connected(start, key)
                        && boost::get(m_incrementalSearchDataBFS->colorMap, u) == boost::default_color_type::white_color)
                    boost::breadth_first_visit(m_adjacencyList, u,
                                               m_incrementalSearchDataBFS->buffer,
                                               m_incrementalSearchDataBFS->visitor,
                                               m_incrementalSearchDataBFS->colorMap);
            }

            for (VertexKey key: m_incrementalSearchDataBFS->fakeNewVertices)
            {
                VertexIndex u = m_vertexIndices.find(key)->second;

                if (Connected(start, key)
                        && boost::get(m_incrementalSearchDataBFS->colorMap, u) == boost::default_color_type::white_color)
                    boost::breadth_first_visit(m_adjacencyList, u,
                                               m_incrementalSearchDataBFS->buffer,
                                               m_incrementalSearchDataBFS->visitor,
                                               m_incrementalSearchDataBFS->colorMap);
            }

            m_incrementalSearchDataBFS->newVertices.clear();
            m_incrementalSearchDataBFS->fakeNewVertices.clear();
            m_incrementalSearchDataBFS->newEdges.clear();
        }
        else if (m_incrementalSearchDataDFS != nullptr)
        {
            m_incrementalSearchDataDFS->start = s;
            try
            {
                boost::depth_first_visit(m_adjacencyList, s,
                                           m_incrementalSearchDataDFS->visitor,
                                           m_incrementalSearchDataDFS->colorMap);
            } catch (EarlyTermination e) { }


            if (m_incrementalSearchDataDFS->connectedOnly)
            {
                for (VertexKey key: m_incrementalSearchDataDFS->newVertices)
                {
                    VertexIndex u = m_vertexIndices.find(key)->second;

                    if (Connected(start, key)
                            && boost::get(m_incrementalSearchDataDFS->colorMap, u) == boost::default_color_type::white_color)
                    {
                        try
                        {
                            boost::depth_first_visit(m_adjacencyList, u,
                                                     m_incrementalSearchDataDFS->visitor,
                                                     m_incrementalSearchDataDFS->colorMap);
                        } catch (EarlyTermination e) { }
                    } // Else already examined or not part of the connected subgraph
                }

                for (VertexKey key: m_incrementalSearchDataDFS->fakeNewVertices)
                {
                    VertexIndex u = m_vertexIndices.find(key)->second;

                    if (Connected(start, key)
                            && boost::get(m_incrementalSearchDataDFS->colorMap, u) == boost::default_color_type::white_color)
                    {
                        try
                        {
                            boost::depth_first_visit(m_adjacencyList, u,
                                                     m_incrementalSearchDataDFS->visitor,
                                                     m_incrementalSearchDataDFS->colorMap);
                        } catch (EarlyTermination e) { }
                    } // Else already examined or not part of the connected subgraph
                }
            }

            m_incrementalSearchDataDFS->newVertices.clear();
            m_incrementalSearchDataDFS->fakeNewVertices.clear();
            m_incrementalSearchDataDFS->newEdges.clear();
        }
    }

    /*!
     * \brief Retrieves all vertices connected to a given vertex.
     * \details The search is performed using a breadth-first search.
     * \param key Vertex key
     * \return Connected vertices, mapping vertex key to data
     */
    std::unordered_map<VertexKey, VertexData> GetConnectedVertices(VertexKey key)
    {
        std::unordered_map<VertexKey, VertexData> connected;

        if (ContainsVertex(key))
        {
            BFSVisitorType visitor;

            visitor.AddAction(BFSVisitorType::Action::DiscoverVertex, [&](VertexIndex v, AdjacencyList g)
            {
                auto &vertex = g[v];
                connected.insert({vertex.key, vertex.data});
            });

            Search(visitor, key);
        }

        return connected;
    }

private:

    typedef struct VertexKeyPairHash
    {
        std::size_t operator()(const std::pair<VertexKey, VertexKey> &key) const
        {
            return (std::hash<VertexKey>{}(key.first) << 1) ^ (std::hash<VertexKey>{}(key.second));
        }
    } VertexKeyPairHash;

    typedef struct SearchCache_BFS
    {
        BFSVisitorType visitor;
        boost::queue<VertexIndex> buffer;
        IndexMap indexMap;
        ColorMap colorMap;
        std::unordered_set<VertexKey> newVertices;

        std::unordered_set<VertexKey> fakeNewVertices;
        std::unordered_set<std::pair<VertexKey, VertexKey>, VertexKeyPairHash> newEdges;
    } SearchCache_BFS;

    typedef struct SearchCache_DFS
    {
        DFSVisitorType visitor;
        boost::queue<VertexIndex> buffer;
        IndexMap indexMap;
        ColorMap colorMap;
        VertexIndex start;
        bool connectedOnly;
        std::unordered_set<VertexKey> newVertices;

        std::unordered_set<VertexKey> fakeNewVertices;
        std::unordered_set<std::pair<VertexKey, VertexKey>, VertexKeyPairHash> newEdges;
    } SearchCache_DFS;

    std::unordered_map<VertexKey, VertexIndex> m_vertexIndices; // User vertex key to underlying vertex index

    AdjacencyList m_adjacencyList; // Underlying graph data

    // Tracks connectivity
    Rank m_rank;
    Parent m_parent;
    RankPMap m_rankPMap;
    ParentPMap m_parentPMap;
    boost::disjoint_sets<RankPMap, ParentPMap> m_disjointSets;

    std::shared_ptr<SearchCache_BFS> m_incrementalSearchDataBFS = nullptr;
    std::shared_ptr<SearchCache_DFS> m_incrementalSearchDataDFS = nullptr;

    /*!
     * \brief Initializes a color map for an incremental search
     * \param colorMap Color map
     */
    void InitializeColorMap(ColorMap &colorMap)
    {
        auto beginEndPair = boost::vertices(m_adjacencyList);
        auto it = beginEndPair.first;

        while (it != beginEndPair.second)
        {
            boost::put(colorMap, *it, boost::default_color_type::white_color);
            ++it;
        }
    }

    BFSVisitorType InitializeIncrementalVisitor(BFSVisitorType userVisitor)
    {
        BFSVisitorType visitor;

        auto fixVertexLambda = [&](typename BFSVisitorType::Action action)
        {
            auto userLambda = userVisitor.GetVertexAction(action);
            if (userLambda != nullptr)
            {
                visitor.AddAction(action, [=](VertexIndex v, AdjacencyList g)
                {
                    if (m_incrementalSearchDataBFS->fakeNewVertices.find(g[v].key) == m_incrementalSearchDataBFS->fakeNewVertices.cend())
                        userLambda(v, g);
                });
            }
        };

        auto fixEdgeLambda = [&](typename BFSVisitorType::Action action)
        {
            auto userLambda = userVisitor.GetEdgeAction(action);
            if (userLambda != nullptr)
            {
                visitor.AddAction(action, [&](EdgeIndex e, AdjacencyList g)
                {
                    bool isDirected = std::is_same<Direction, Directed>();

                    VertexIndex u = boost::source(e, g);
                    VertexIndex v = boost::target(e, g);

                    bool fakeNewU = m_incrementalSearchDataBFS->fakeNewVertices.find(g[u].key) == m_incrementalSearchDataBFS->fakeNewVertices.cend();
                    bool newEdgeUV = m_incrementalSearchDataBFS->newEdges.find({g[u].key, g[v].key}) == m_incrementalSearchDataBFS->newEdges.cend();

                    if (isDirected)
                    {
                        if (!fakeNewU)
                            userLambda(e, g);
                        else if (newEdgeUV)
                            userLambda(e, g);
                    }
                    else
                    {
                        bool fakeNewV = m_incrementalSearchDataBFS->fakeNewVertices.find(g[v].key) == m_incrementalSearchDataBFS->fakeNewVertices.cend();
                        bool newEdgeVU = m_incrementalSearchDataBFS->newEdges.find({g[v].key, g[u].key}) == m_incrementalSearchDataBFS->newEdges.cend();

                        if (!fakeNewU || !fakeNewV)
                            userLambda(e, g);
                        else if (newEdgeUV || newEdgeVU)
                            userLambda(e, g);
                    }
                });
            }
        };

        fixVertexLambda(BFSVisitorType::Action::DiscoverVertex);
        fixVertexLambda(BFSVisitorType::Action::ExamineVertex);
        fixVertexLambda(BFSVisitorType::Action::FinishVertex);
        fixEdgeLambda(BFSVisitorType::Action::ExamineEdge);
        fixEdgeLambda(BFSVisitorType::Action::TreeEdge);
        fixEdgeLambda(BFSVisitorType::Action::NontreeEdge);
        fixEdgeLambda(BFSVisitorType::Action::GrayTarget);
        fixEdgeLambda(BFSVisitorType::Action::BlackTarget);

        return visitor;
    }

    DFSVisitorType InitializeIncrementalVisitor(DFSVisitorType userVisitor)
    {
        DFSVisitorType visitor;

        auto fixVertexLambda = [&](typename DFSVisitorType::Action action)
        {
            auto userLambda = userVisitor.GetVertexAction(action);
            if (userLambda != nullptr)
            {
                visitor.AddAction(action, [=](VertexIndex v, AdjacencyList g)
                {
                    if (m_incrementalSearchDataDFS->fakeNewVertices.find(g[v].key) == m_incrementalSearchDataDFS->fakeNewVertices.cend())
                        userLambda(v, g);
                });
            }
        };

        auto fixEdgeLambda = [&](typename DFSVisitorType::Action action)
        {
            auto userLambda = userVisitor.GetEdgeAction(action);
            if (userLambda != nullptr)
            {
                visitor.AddAction(action, [&](EdgeIndex e, AdjacencyList g)
                {
                    bool isDirected = std::is_same<Direction, Directed>();

                    VertexIndex u = boost::source(e, g);
                    VertexIndex v = boost::target(e, g);

                    bool fakeNewU = m_incrementalSearchDataDFS->fakeNewVertices.find(g[u].key) == m_incrementalSearchDataDFS->fakeNewVertices.cend();
                    bool newEdgeUV = m_incrementalSearchDataDFS->newEdges.find({g[u].key, g[v].key}) == m_incrementalSearchDataDFS->newEdges.cend();

                    if (isDirected)
                    {
                        if (!fakeNewU)
                            userLambda(e, g);
                        else if (newEdgeUV)
                            userLambda(e, g);
                    }
                    else
                    {
                        bool fakeNewV = m_incrementalSearchDataDFS->fakeNewVertices.find(g[v].key) == m_incrementalSearchDataDFS->fakeNewVertices.cend();
                        bool newEdgeVU = m_incrementalSearchDataDFS->newEdges.find({g[v].key, g[u].key}) == m_incrementalSearchDataDFS->newEdges.cend();

                        if (!fakeNewU || !fakeNewV)
                            userLambda(e, g);
                        else if (newEdgeUV || newEdgeVU)
                            userLambda(e, g);
                    }
                });
            }
        };

        fixVertexLambda(DFSVisitorType::Action::StartVertex);
        fixVertexLambda(DFSVisitorType::Action::DiscoverVertex);
        fixVertexLambda(DFSVisitorType::Action::FinishVertex);
        fixEdgeLambda(DFSVisitorType::Action::ExamineEdge);
        fixEdgeLambda(DFSVisitorType::Action::TreeEdge);
        fixEdgeLambda(DFSVisitorType::Action::BackEdge);
        fixEdgeLambda(DFSVisitorType::Action::ForwardOrCrossEdge);
        fixEdgeLambda(DFSVisitorType::Action::FinishEdge);

        return visitor;
    }
};

}
}
#endif // GRAPH_H
