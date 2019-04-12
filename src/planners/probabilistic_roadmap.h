#ifndef PROBABILISTIC_ROADMAP_H
#define PROBABILISTIC_ROADMAP_H

//#include <boost/graph/graph_traits.hpp>
//#include <boost/graph/adjacency_list.hpp>

#include "base/state_space/state.h"
#include "base/math/cost.h"

namespace mace {
namespace planners{

//class PRM
//{
//public:
//    PRM();

//private:

//    struct vertex_state_t
//     {
//         typedef boost::vertex_property_tag kind;
//     };

//     struct vertex_total_connection_attempts_t
//     {
//         typedef boost::vertex_property_tag kind;
//     };

//     struct vertex_successful_connection_attempts_t
//     {
//         typedef boost::vertex_property_tag kind;
//     };

//    typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;

//    typedef boost::property<vertex_state_t, state_space::State*> VertexProperty;

//    typedef boost::adjacency_list<
//        boost::vecS, boost::vecS, boost::undirectedS,
//        VertexProperty, EdgeWeightProperty> Graph;

//     Graph g_;

//    typedef boost::graph_traits<Graph>::vertex_descriptor vertex;
//    typedef boost::graph_traits<Graph>::edge_descriptor edge;
//    typedef boost::property_map<Graph,vertex_state_t>::type stateProperty;

//     void addMilestone(state_space::State* state);

//};

} //end of namespace planners
} //end of namespace mace

#endif // PROBABILISTIC_ROADMAP_H
