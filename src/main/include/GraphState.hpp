/**
 * @file
 * @author Massimo Bono
 * @brief definition of every involving the a state of a gridmap which search is time dependent
 * @version 0.1
 * @date 2019-09-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef _PATH_FINDING_UTILS_GRAPH_STATE_HEADER__
#define _PATH_FINDING_UTILS_GRAPH_STATE_HEADER__

#include "ISearchState.hpp"
#include "IStateSupplier.hpp"
#include "IStateExpander.hpp"
#include "IGoalChecker.hpp"
#include <cpp-utils/StaticPriorityQueue.hpp>
#include <cpp-utils/igraph.hpp>
#include <cpp-utils/mapplus.hpp>
#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/adjacentGraph.hpp>
#include "AbstractGraphState.hpp"
#include "AbstractSimpleWeightedDirectedGraphStateSupplier.hpp"
#include <tuple>
#include <functional>
#include <boost/compute/functional/identity.hpp>
#include <cpp-utils/functional.hpp>

namespace pathfinding::search {

    using namespace pathfinding;
    using namespace cpp_utils::graphs;

    /**
     * @brief a state that represents a vertex in a graph which references the graph explicitly inside it.
     * 
     * It's a better version of GraphlessState, however suince it contains the reference of the graph, it is harder to instantiate
     * 
     */
    template <typename G, typename V, typename E = cost_t>
    class GraphState: public AbstractGraphState<V> {
        typedef GraphState<G,V,E> GraphStateInstance;
    public:
        using AbstractGraphState<V>::getPayload;
    protected:
        /**
         * @brief the graph where the state will be located
         */
        const cpp_utils::graphs::IImmutableGraph<G, V, E>& graph;
    public:
        GraphState(stateid_t id, const IImmutableGraph<G, V, E>& graph, nodeid_t position): AbstractGraphState<V>{id, position}, graph{graph} {

        }

        GraphState(const GraphStateInstance& other): AbstractGraphState<V>{other}, graph{other.graph} {
        }
        GraphStateInstance& operator =(const GraphStateInstance& other) {
            assert(this->graph == other.graph);
            AbstractGraphState<V>::operator =(other);
            this->graph = other.graph;
            return *this;
        }
        GraphState(GraphStateInstance&& other): AbstractGraphState<V>{::std::move(other)}, graph{other.graph} {
        }
        GraphStateInstance& operator =(GraphStateInstance&& other) {
            assert(this->graph == other.graph);
            AbstractGraphState<V>::operator =(::std::move(other));
            this->graph = other.graph;
            return *this;
        }

        GraphStateInstance* getParent() {
            return static_cast<GraphStateInstance*>(this->parent); 
        }

        const GraphStateInstance* getParent() const {
            return static_cast<const GraphStateInstance*>(this->parent); 
        }
        
        /**
         * @brief get a tuple containing a set of values which allows you to human identify the state
         * 
         * @return a tuple containing n the first index the node id in the graph ajnd the second value is the timestamp
         */
        virtual std::tuple<const V&> getPayload() const {
            return std::tuple<const V&>{this->graph.getVertex(this->getPosition())};
        }
    public:
        const cpp_utils::graphs::IImmutableGraph<G, V, E>& getGraph() const {
            return this->graph;
        }
    public:
        friend std::ostream& operator <<(std::ostream& out, const GraphStateInstance& state) {
            out 
                << "{id: " 
                << state.getId() 
                << " node: " 
                << state.graph.getVertex(state.getPosition())
                << "}";
            return out;
        }
    public:
        MemoryConsumption getByteMemoryOccupied() const {
            return sizeof(*this);
        }
    };

    /**
     * @brief A supplier which generate as many search nodes as there are vertices in the map
     * 
     * @tparam G payload type pf the map
     * @tparam V payload type of each vertex in the graph
     */
    template <typename G, typename V, typename E = cost_t>
    class GraphStateSupplier: public AbstractSimpleWeightedDirectedGraphStateSupplier<GraphState<G, V, E>, G, V, E> {
        typedef GraphStateSupplier<G, V, E> GraphStateSupplierInstance;
    protected:
        virtual stateid_t generateStateId(nodeid_t location) {
            return location;
        }

        virtual GraphState<G, V, E> generateNewInstance(stateid_t id, nodeid_t location) {
            return GraphState<G, V, E>{id, this->graph, location};
        }
    public:
        /**
         * @brief Construct a new Graph State Supplier object
         * 
         * @param graph the graph that will be blindly for each search node we're going to create
         */
        GraphStateSupplier(const IImmutableGraph<G, V, E>& graph): AbstractSimpleWeightedDirectedGraphStateSupplier<GraphState<G, V, E>, G, V, E>{graph} {

        }
        GraphStateSupplier(GraphStateSupplierInstance&& other): AbstractSimpleWeightedDirectedGraphStateSupplier<GraphState<G, V, E>, G, V, E>{::std::move(other)} {
            
        }
        ~GraphStateSupplier() {

        }
        GraphStateSupplierInstance& operator =(GraphStateSupplierInstance&& other) {
            AbstractSimpleWeightedDirectedGraphStateSupplier<GraphState<G, V, E>, G, V, E>::operator=(::std::move(other));
            return *this;
        }
    };


}


#endif