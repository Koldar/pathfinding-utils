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
    template <typename G, typename V, typename E, typename REASON>
    class GraphState: public AbstractGraphState<REASON, V> {
        using This = GraphState<G, V, E, REASON>;
        using Super = AbstractGraphState<REASON, V>;
    public:
        using Super::getPayload;
    protected:
        /**
         * @brief the graph where the state will be located
         */
        const cpp_utils::graphs::IImmutableGraph<G, V, E>& graph;
    public:
        GraphState(stateid_t id, const IImmutableGraph<G, V, E>& graph, nodeid_t position, const REASON& reason): Super{id, position, reason}, graph{graph} {

        }

        GraphState(const This& other): Super{other}, graph{other.graph} {
        }
        This& operator =(const This& other) {
            assert(this->graph == other.graph);
            Super::operator =(other);
            this->graph = other.graph;
            return *this;
        }
        GraphState(This&& other): Super{::std::move(other)}, graph{other.graph} {
        }
        This& operator =(This&& other) {
            assert(this->graph == other.graph);
            Super::operator =(::std::move(other));
            this->graph = other.graph;
            return *this;
        }

        This* getParent() {
            return static_cast<This*>(this->parent); 
        }

        const This* getParent() const {
            return static_cast<const This*>(this->parent); 
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
        friend std::ostream& operator <<(std::ostream& out, const This& state) {
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
    template <typename G, typename V, typename E = cost_t, typename REASON>
    class GraphStateSupplier: public AbstractSimpleWeightedDirectedGraphStateSupplier<GraphState<G, V, E, REASON>, G, V, E, REASON> {
        using State = GraphState<G, V, E, REASON>;
        using This = GraphStateSupplier<G, V, E>;
        using Super = AbstractSimpleWeightedDirectedGraphStateSupplier<State, G, V, E, REASON>;
    protected:
        virtual stateid_t generateStateId(nodeid_t location, const REASON& reason) {
            return location;
        }

        virtual State generateNewInstance(stateid_t id, nodeid_t location, const REASON& reason) {
            return State{id, this->graph, location, reason};
        }
    public:
        /**
         * @brief Construct a new Graph State Supplier object
         * 
         * @param graph the graph that will be blindly for each search node we're going to create
         */
        GraphStateSupplier(const IImmutableGraph<G, V, E>& graph): Super{graph} {
            debug("GraphStateSupplier called");
        }
        GraphStateSupplier(const This& other) = delete;
        GraphStateSupplier(This&& other): Super{::std::move(other)} {
            debug("This MOVED");
        }
        virtual ~GraphStateSupplier() {
            debug("GraphStateSupplier destroyed!");
        }
        This& operator =(This&& other) {
            Super::operator=(::std::move(other));
            debug("This MOVED");
            return *this;
        }
        This& operator =(const This& other) = delete;
    };


}


#endif