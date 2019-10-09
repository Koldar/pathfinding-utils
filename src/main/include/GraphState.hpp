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
    class GraphState: public AbstractGraphState<GraphState<G,V>, V> {
        typedef GraphState<G,V,E> GraphStateInstance;
    protected:
        /**
         * @brief the graph where the state will be located
         */
        const cpp_utils::graphs::IImmutableGraph<G, V, E>& graph;
    public:
        GraphState(stateid_t id, const IImmutableGraph<G, V, E>& graph, nodeid_t position): AbstractGraphState<GraphStateInstance, V>{id, position}, graph{graph} {

        }

        GraphState(const GraphStateInstance& other): AbstractGraphState<GraphStateInstance, V>{other}, graph{other.graph} {
        }
        GraphStateInstance& operator =(const GraphStateInstance& other) {
            assert(this->graph == other.graph);
            AbstractGraphState<GraphState<G,V>, V>::operator =(other);
            this->graph = other.graph;
            return *this;
        }
        GraphState(GraphStateInstance&& other): AbstractGraphState<GraphStateInstance, V>{::std::move(other)}, graph{other.graph} {
        }
        GraphStateInstance& operator =(GraphStateInstance&& other) {
            assert(this->graph == other.graph);
            AbstractGraphState<GraphStateInstance, V>::operator =(::std::move(other));
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
        std::tuple<const V&> getPayload() const {
            return std::tuple<const V&>{this->graph.getVertex(this->getPosition())};
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

    /**
     * @brief allows to generate the successors of a GraphState
     * 
     * @tparam G payload of the underlying weighted directed graph
     */
    template <typename G, typename V, typename E=cost_t, typename GET_COST=boost::compute::identity<cost_t>()>
    class GraphStateExpander: public IStateExpander<GraphState<G,V>, nodeid_t> {
    private:
        const cpp_utils::graphs::IImmutableGraph<G, V, E>& graph;
    public:
        GraphStateExpander(const cpp_utils::graphs::IImmutableGraph<G, V, E>& graph): graph{graph} {

        }
        virtual ~GraphStateExpander() {

        }
        GraphStateExpander(const GraphStateExpander<G, V>& other) : graph{other.graph} {

        }
        GraphStateExpander<G, V>& operator =(const GraphStateExpander<G, V>& other) = delete;
        GraphStateExpander(GraphStateExpander<G, V>&& other) : graph{other.graph} {

        }
        GraphStateExpander<G, V>& operator =(GraphStateExpander<G, V>&& other) {
            this->graph = other.graph;
            return *this;
        }
    public:
        virtual cpp_utils::vectorplus<std::pair<GraphState<G,V>&, cost_t>> getSuccessors(const GraphState<G,V>& state, IStateSupplier<GraphState<G,V>, nodeid_t>& supplier) {
            cpp_utils::vectorplus<std::pair<GraphState<G,V>&, cost_t>> result{};
            //****************** MOVING *********************
            for (auto outEdge : graph.getOutEdges(state.getPosition())) {
                fine("an outedge ", outEdge, " of ", state, "(", &state, ") goes to", outEdge.getSinkId(), "edge payload of", outEdge.getPayload());
                result.add(std::pair<GraphState<G,V>&, cost_t>{
                    supplier.getState(outEdge.getSinkId()),
                    GET_COST(outEdge.getPayload())
                });
            }

            return result;
        }
        virtual std::pair<GraphState<G,V>&, cost_t> getSuccessor(const GraphState<G,V>& state, int successorNumber, IStateSupplier<GraphState<G,V>, nodeid_t>& supplier) {
            auto outEdge = this->graph.getOutEdge(state.getPosition(), successorNumber);
            return std::pair<GraphState<G,V>&, cost_t>{
                supplier.getState(outEdge.getSinkId()),
                GET_COST(outEdge.getPayload())
            };
        }
    public:
        virtual void cleanup() {

        }
    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            return sizeof(*this);
        }
    };

    /**
     * @brief true if the state position is the same as the goal one
     * 
     */
    template <typename G, typename V>
    class GraphStateGoalChecker: public IGoalChecker<GraphState<G, V>> {
    public:
        virtual bool isGoal(const GraphState<G,V>& state, const GraphState<G,V>* goal) const {
            return state.getPosition() == goal->getPosition();
        }
    public:
        virtual void cleanup() {

        }
    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            return sizeof(*this);
        }
};



}


#endif