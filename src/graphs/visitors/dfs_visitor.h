#ifndef DFS_VISITOR_H
#define DFS_VISITOR_H

#include "common/common.h"

#include <unordered_map>

#include <boost/graph/visitors.hpp>
#include <boost/graph/depth_first_search.hpp>


#include <functional>

namespace mace {
namespace graphs {

/*!
 * \brief The DFSVisitor class defines a visitor to be used in a depth-first search for a graph.
 */
template <typename AdjacencyList, typename VertexIndex, typename EdgeIndex>
class DFSVisitor : public boost::default_dfs_visitor
{
public:
    /*!
     * \brief The Action enum specifies every point that this class may be invoked during the search
     */
    typedef enum class Action
    {
        InitializeVertex, /*!< Invoked on the start vertex prior to starting the search */
        StartVertex, /*!< Invoked on the start vertex prior to starting the search */
        DiscoverVertex, /*!< Invoked when a vertex is discovered */
        ExamineEdge, /*!< Invoked when an edge is examined */
        TreeEdge, /*!< Invoked when an edge becomes an edge forming the search tree */
        BackEdge, /*!< Invoked on the back edges in the graph */
        ForwardOrCrossEdge, /*!< Invoked on the forward or cross edges in the graph */
        FinishEdge, /*!< Invoked when the edge is finished being examined */
        FinishVertex /*!< Invoked on a vertex U after finishing all vertices in the search tree rooted at U */
    } Action;

    /*!
     * \brief Constructor
     */
    DFSVisitor()
    {
    }

    /*!
     * \brief Adds a lambda to be performed on vertices when some action occurs
     * \param action Action type
     * \param lambda Lambda
     */
    void AddAction(Action action, std::function<void(VertexIndex, AdjacencyList)> lambda)
    {
        switch (action)
        {
            case Action::InitializeVertex:
                m_initializeVertexLambda = lambda;
                break;
            case Action::StartVertex:
                m_startVertexLambda = lambda;
                break;
            case Action::DiscoverVertex:
                m_discoverVertexLambda = lambda;
                break;
            case Action::FinishVertex:
                m_finishVertexLambda = lambda;
                break;
            default:
                throw std::runtime_error("Unsupported action type to invoke function on vertex");
                break;
        }
    }

    /*!
     * \brief Retrieves the lambda to be invoked on the given vertex action
     * \param action Action
     * \return Lambda to be invoked on the given action
     */
    std::function<void(VertexIndex, AdjacencyList)> GetVertexAction(Action action)
    {
        switch (action)
        {
            case Action::InitializeVertex:
                return m_initializeVertexLambda;
            case Action::StartVertex:
                return m_startVertexLambda;
            case Action::DiscoverVertex:
                return m_discoverVertexLambda;
            case Action::FinishVertex:
                return m_finishVertexLambda;
            default:
                throw std::runtime_error("Unsupported action type to invoke function on vertex");
                break;
        }
    }

    /*!
     * \brief Adds a lambda to be performed on edges when some action occurs
     * \param action Action type
     * \param lambda Lambda
     */
    void AddAction(Action action, std::function<void(EdgeIndex, AdjacencyList)> lambda)
    {
        switch (action)
        {
            case Action::ExamineEdge:
                m_examineEdgeLambda = lambda;
                break;
            case Action::TreeEdge:
                m_treeEdgeLambda = lambda;
                break;
            case Action::BackEdge:
                m_backEdgeLambda = lambda;
                break;
            case Action::ForwardOrCrossEdge:
                m_forwardOrCrossEdgeLambda = lambda;
                break;
            case Action::FinishEdge:
                m_finishEdgeLambda = lambda;
                break;
            default:
                throw std::runtime_error("Unsupported action type to invoke function on edge");
                break;
        }
    }

    /*!
     * \brief Retrieves the lambda to be invoked on the given edge action
     * \param action Action
     * \return Lambda to be invoked on the given action
     */
    std::function<void(EdgeIndex, AdjacencyList)> GetEdgeAction(Action action)
    {
        switch (action)
        {
            case Action::ExamineEdge:
                return m_examineEdgeLambda;
            case Action::TreeEdge:
                return m_treeEdgeLambda;
            case Action::BackEdge:
                return m_backEdgeLambda;
            case Action::ForwardOrCrossEdge:
                return m_forwardOrCrossEdgeLambda;
            case Action::FinishEdge:
                return m_finishEdgeLambda;
            default:
                throw std::runtime_error("Unsupported action type to invoke function on edge");
                break;
        }
        return nullptr;
    }

    /*!
     * \brief Invoked on every vertex during initialization
     * \param v Vertex index
     * \param g Adjacency list
     */
    void initialize_vertex(VertexIndex v, AdjacencyList g)
    {
        if (m_initializeVertexLambda != nullptr)
            m_initializeVertexLambda(v, g);
    }

    /*!
     * \brief Invoked on the start vertex prior to starting the search
     * \param v Vertex index
     * \param g Adjacency list
     */
    void start_vertex(VertexIndex v, AdjacencyList g)
    {
        if (m_startVertexLambda != nullptr)
            m_startVertexLambda(v, g);
    }

    /*!
     * \brief Invoked when a vertex is discovered
     * \param v Vertex index
     * \param g Adjacency list
     */
    void discover_vertex(VertexIndex v, AdjacencyList g)
    {
        if (m_discoverVertexLambda != nullptr)
            m_discoverVertexLambda(v, g);
    }

    /*!
     * \brief Invoked when an edge is examined
     * \param e Edge index
     * \param g Adjacency list
     */
    void examine_edge(EdgeIndex e, AdjacencyList g)
    {
        if (m_examineEdgeLambda != nullptr)
            m_examineEdgeLambda(e, g);
    }

    /*!
     * \brief Invoked when an edge becomes an edge forming the search tree
     * \param e Edge index
     * \param g Adjacency list
     */
    void tree_edge(EdgeIndex e, AdjacencyList g)
    {
        if (m_treeEdgeLambda != nullptr)
            m_treeEdgeLambda(e, g);
    }

    /*!
     * \brief Invoked on the back edges in the graph
     * \details Note that in an undirected graph, both tree_edge() and back_edge() will be invoked. tree_edge() will be invoked first
     * \param e Edge index
     * \param g Adjacency list
     */
    void back_edge(EdgeIndex e, AdjacencyList g)
    {
        if (m_backEdgeLambda != nullptr)
            m_backEdgeLambda(e, g);
    }

    /*!
     * \brief Invoked on the forward or cross edges in the graph
     * \details Never called in an undirected graph
     * \param e Edge index
     * \param g Adjacency list
     */
    void forward_or_cross_edge(EdgeIndex e, AdjacencyList g)
    {
        if (m_forwardOrCrossEdgeLambda != nullptr)
            m_forwardOrCrossEdgeLambda(e, g);
    }

    /*!
     * \brief Invoked when the edge is finished being examined
     * \param e Edge index
     * \param g Adjacency list
     */
    void finish_edge(EdgeIndex e, AdjacencyList g)
    {
        if (m_finishEdgeLambda != nullptr)
            m_finishEdgeLambda(e, g);
    }

    /*!
     * \brief Invoked on a vertex U after finishing all vertices in the search tree rooted at U
     * \details If U is a leaf vertex, this is invoked after examining all of its out edges
     * \param v Vertex index
     * \param g Adjacency list
     */
    void finish_vertex(VertexIndex v, AdjacencyList g)
    {
        if (m_finishVertexLambda != nullptr)
            m_finishVertexLambda(v, g);
    }

private:
    std::function<void(VertexIndex, AdjacencyList)> m_initializeVertexLambda = nullptr;
    std::function<void(VertexIndex, AdjacencyList)> m_startVertexLambda = nullptr;
    std::function<void(VertexIndex, AdjacencyList)> m_discoverVertexLambda = nullptr;
    std::function<void(EdgeIndex, AdjacencyList)> m_examineEdgeLambda = nullptr;
    std::function<void(EdgeIndex, AdjacencyList)> m_treeEdgeLambda = nullptr;
    std::function<void(EdgeIndex, AdjacencyList)> m_backEdgeLambda = nullptr;
    std::function<void(EdgeIndex, AdjacencyList)> m_forwardOrCrossEdgeLambda = nullptr;
    std::function<void(EdgeIndex, AdjacencyList)> m_finishEdgeLambda = nullptr;
    std::function<void(VertexIndex, AdjacencyList)> m_finishVertexLambda = nullptr;
};

}
}
#endif // DFS_VISITOR_H
