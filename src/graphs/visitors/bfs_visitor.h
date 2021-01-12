#ifndef VISITORSFS_VISITOR_H
#define VISITORSFS_VISITOR_H

#include "common/common.h"

#include <unordered_map>

#include <boost/graph/visitors.hpp>
#include <boost/graph/breadth_first_search.hpp>

namespace mace {
namespace graphs {

/*!
 * \brief The BFSVisitor class defines a visitor to be used in a breadth-first search for a graph.
 */
template <typename AdjacencyList, typename VertexIndex, typename EdgeIndex>
class BFSVisitor : public boost::default_bfs_visitor
{
public:
    /*!
     * \brief The Action enum specifies every point that this class may be invoked during the search
     */
    typedef enum class Action
    {
        InitializeVertex, /*!< Invoked at each vertex during initialization */
        DiscoverVertex, /*!< Invoked at each vertex when it is first discovered */
        ExamineVertex, /*!< Invoked at each vertex when it is examined. Occurs immediately before any out-edges are examined. */
        ExamineEdge, /*!< Invoked at each edge when it is first examined */
        TreeEdge, /*!< Invoked at each edge if it becomes an edge forming the search tree */
        NontreeEdge, /*!< Invoked at each cross edge*/
        GrayTarget, /*!< Invoked when an edge's target vertex is colored gray. Gray indicates the vertex is queued to be examined */
        BlackTarget, /*!< Invoked when an edge's target vertex is colored black. Black indicates the vertex has already been examined */
        FinishVertex /*!< Invoked after examining a vertex and discovering all adjacent vertices, but before the out-edges are examined */
    } Action;

    /*!
     * \brief Constructor
     */
    BFSVisitor()
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
            case Action::DiscoverVertex:
                m_discoverVertexLambda = lambda;
                break;
            case Action::ExamineVertex:
                m_examineVertexLambda = lambda;
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
            case Action::DiscoverVertex:
                return m_discoverVertexLambda;
            case Action::ExamineVertex:
                return m_examineVertexLambda;
            case Action::FinishVertex:
                return m_finishVertexLambda;
            default:
                throw std::runtime_error("Unsupported action type to invoke function on vertex");
                break;
        }
        return nullptr;
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
            case Action::NontreeEdge:
                m_nontreeEdgeLambda = lambda;
                break;
            case Action::GrayTarget:
                m_grayTargetLambda = lambda;
                break;
            case Action::BlackTarget:
                m_blackTargetLambda = lambda;
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
            case Action::NontreeEdge:
                return m_nontreeEdgeLambda;
            case Action::GrayTarget:
                return m_grayTargetLambda;
            case Action::BlackTarget:
                return m_blackTargetLambda;
            default:
                throw std::runtime_error("Unsupported action type to invoke function on edge");
                break;
        }
        return nullptr;
    }

    /*!
     * \brief Invoked at each vertex during initialization
     * \param v Vertex index
     * \param g Adjacency list
     */
    void initialize_vertex(VertexIndex v, AdjacencyList g)
    {
        if (m_initializeVertexLambda != nullptr)
            m_initializeVertexLambda(v, g);
    }

    /*!
     * \brief Invoked at each vertex when it is first discovered
     * \param v Vertex index
     * \param g Adjacency list
     */
    void discover_vertex(VertexIndex v, AdjacencyList g)
    {
        if (m_discoverVertexLambda != nullptr)
            m_discoverVertexLambda(v, g);
    }

    /*!
     * \brief Invoked at each vertex when it is examined. Occurs immediately before any out-edges are examined.
     * \param v Vertex index
     * \param g Adjacency list
     */
    void examine_vertex(VertexIndex v, AdjacencyList g)
    {
        if (m_examineVertexLambda != nullptr)
            m_examineVertexLambda(v, g);
    }

    /*!
     * \brief  Invoked at each edge when it is first examined
     * \param e Edge index
     * \param g Adjacency list
     */
    void examine_edge(EdgeIndex e, AdjacencyList g)
    {
        if (m_examineEdgeLambda != nullptr)
            m_examineEdgeLambda(e, g);
    }

    /*!
     * \brief Invoked at each edge if it becomes an edge forming the search tree
     * \param e Edge index
     * \param g Adjacency list
     */
    void tree_edge(EdgeIndex e, AdjacencyList g)
    {
        if (m_treeEdgeLambda != nullptr)
            m_treeEdgeLambda(e, g);
    }

    /*!
     * \brief Invoked at each cross edge
     * \param e Edge index
     * \param g Adjacency list
     */
    void non_tree_edge(EdgeIndex e, AdjacencyList g)
    {
        if (m_nontreeEdgeLambda != nullptr)
            m_nontreeEdgeLambda(e, g);
    }

    /*!
     * \brief Invoked when an edge's target vertex is colored gray. Gray indicates the vertex is queued to be examined
     * \param e Edge index
     * \param g Adjacency list
     */
    void gray_target(EdgeIndex e, AdjacencyList g)
    {
        if (m_grayTargetLambda != nullptr)
            m_grayTargetLambda(e, g);
    }

    /*!
     * \brief Invoked when an edge's target vertex is colored black. Black indicates the vertex has already been examined
     * \param e Edge index
     * \param g Adjacency list
     */
    void black_target(EdgeIndex e, AdjacencyList g)
    {
        if (m_blackTargetLambda != nullptr)
            m_blackTargetLambda(e, g);
    }

    /*!
     * \brief Invoked after examining a vertex and discovering all adjacent vertices, but before the out-edges are examined
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
    std::function<void(VertexIndex, AdjacencyList)> m_discoverVertexLambda = nullptr;
    std::function<void(VertexIndex, AdjacencyList)> m_examineVertexLambda = nullptr;
    std::function<void(EdgeIndex, AdjacencyList)> m_examineEdgeLambda = nullptr;
    std::function<void(EdgeIndex, AdjacencyList)> m_treeEdgeLambda = nullptr;
    std::function<void(EdgeIndex, AdjacencyList)> m_nontreeEdgeLambda = nullptr;
    std::function<void(EdgeIndex, AdjacencyList)> m_grayTargetLambda = nullptr;
    std::function<void(EdgeIndex, AdjacencyList)> m_blackTargetLambda = nullptr;
    std::function<void(VertexIndex, AdjacencyList)> m_finishVertexLambda = nullptr;
};

}
}
#endif // VISITORSFS_VISITOR_H
