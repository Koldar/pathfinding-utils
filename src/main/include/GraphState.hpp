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
#ifndef _CPD_SEARCH_GRIDMAPSTATE_HEADER__
#define _CPD_SEARCH_GRIDMAPSTATE_HEADER__

#include "ISearchState.hpp"
#include "IStateSupplier.hpp"
#include "IStateExpander.hpp"
#include "IGoalChecker.hpp"
#include "GraphlessState.hpp"
#include <cpp-utils/StaticPriorityQueue.hpp>
#include <cpp-utils/igraph.hpp>
#include <cpp-utils/mapplus.hpp>
#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/adjacentGraph.hpp>
#include "AbstractSimpleWeightedDirectedGraphStateSupplier.hpp"
#include <tuple>

namespace pathfinding::search {

    using namespace pathfinding;
    using namespace cpp_utils::graphs;

    /**
     * @brief a state that represents a vertex in a graph which references the graph explicitly inside it.
     * 
     * It's a better version of GraphlessState, however suince it contains the reference of the graph, it is harder to instantiate
     * 
     */
    template <typename G, typename V>
    class GraphState: public AbstractGraphState<GraphState<G,V>, V> {
    protected:
        /**
         * @brief the graph where the state will be located
         */
        const cpp_utils::graphs::IImmutableGraph<G, V, cost_t>& graph;
    public:
        GraphState(stateid_t id, const IImmutableGraph<G, V, cost_t>& graph, nodeid_t position): AbstractGraphState<GraphState<G,V>, V>{id, position}, graph{graph} {

        }

        GraphState(const GraphState<G,V>& other): AbstractGraphState<GraphState<G,V>, V>{other}, graph{other.graph} {
        }
        GraphState<G, V>& operator =(const GraphState<G,V>& other) {
            assert(this->graph == other.graph);
            AbstractGraphState<GraphState<G,V>, V>::operator =(other);
            this->graph = other.graph;
            return *this;
        }
        GraphState(GraphState<G, V>&& other): AbstractGraphState<GraphState<G,V>, V>{::std::move(other)}, graph{other.graph} {
        }
        GraphState<G, V>& operator =(GraphState<G,V>&& other) {
            assert(this->graph == other.graph);
            AbstractGraphState<GraphState<G,V>, V>::operator =(::std::move(other));
            this->graph = other.graph;
            return *this;
        }

        GraphState<G,V>* getParent() {
            return static_cast<GraphState<G,V>*>(this->parent); 
        }

        const GraphState<G,V>* getParent() const {
            return static_cast<const GraphState<G,V>*>(this->parent); 
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
        friend std::ostream& operator <<(std::ostream& out, const GraphState<G,V>& state) {
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
    template <typename G, typename V>
    class GraphStateSupplier: public AbstractSimpleWeightedDirectedGraphStateSupplier<GraphState<G, V>, G, V> {
    protected:
        virtual stateid_t generateStateId(nodeid_t location) {
            return location;
        }

        virtual GraphState<G,V> generateNewInstance(stateid_t id, nodeid_t location) {
            return GraphState<G,V>{id, location, this->graph.getVertex(location)};
        }
    public:
        GraphStateSupplier(const IImmutableGraph<G, V, cost_t>& graph): AbstractSimpleWeightedDirectedGraphStateSupplier<GraphState<G,V>, G, V>{graph} {

        }
        GraphStateSupplier(GraphStateSupplier<G,V>&& other): AbstractSimpleWeightedDirectedGraphStateSupplier<GraphState<G,V>, G, V>{::std::move(other)} {
            
        }
        ~GraphStateSupplier() {

        }
        GraphStateSupplier<G,V>& operator =(GraphStateSupplier<G,V>&& other) {
            AbstractSimpleWeightedDirectedGraphStateSupplier<GraphState<G,V>, G, V>::operator=(::std::move(other));
            return *this;
        }
    };

    /**
     * @brief allows to generate the successors of a GraphState
     * 
     * @tparam G payload of the underlying weighted directed graph
     */
    template <typename G, typename V>
    class GraphStateExpander: public IStateExpander<GraphState<G,V>, nodeid_t> {
    private:
        const cpp_utils::graphs::IImmutableGraph<G, V, cost_t>& graph;
    public:
        GraphStateExpander(const cpp_utils::graphs::IImmutableGraph<G, V, cost_t>& graph): graph{graph} {

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
                    outEdge.getPayload()
                });
            }

            return result;
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