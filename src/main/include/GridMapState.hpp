#ifndef _GRIDMAPSTATE_HEADER__
#define _GRIDMAPSTATE_HEADER__

#include "ISearchAlgorithm.hpp"
#include "ISearchState.hpp"
#include "GridBranching.hpp"
#include "GraphState.hpp"
#include "IStateSupplier.hpp"
#include "IStateExpander.hpp"
#include "IGoalChecker.hpp"
#include "xyLoc.hpp"
#include "GraphlessState.hpp"
#include "AbstractSimpleWeightedDirectedGraphStateSupplier.hpp"
#include <cpp-utils/StaticPriorityQueue.hpp>
#include <cpp-utils/igraph.hpp>
#include <cpp-utils/vectorplus.hpp>
#include <utility>

namespace pathfinding::search {

    /**
     * @brief a search state over a map we know it's a grid map
     * 
     * In this state there is only x and y coordinates
     * 
     */
    using GridMapState = GraphlessState<xyLoc>;

    template <typename G>
    class GridMapStateSupplier: public AbstractSimpleWeightedDirectedGraphStateSupplier<GridMapState, G, xyLoc, cost_t> {
    protected:
        virtual stateid_t generateStateId(nodeid_t location) {
            return location;
        }

        virtual GridMapState generateNewInstance(stateid_t id, nodeid_t location) {
            return GridMapState{id, location, this->graph.getVertex(location)};
        }
    public:
        GridMapStateSupplier(const IImmutableGraph<G, xyLoc, cost_t>& graph): AbstractSimpleWeightedDirectedGraphStateSupplier<GridMapState, G, xyLoc, cost_t>{graph} {

        }
        GridMapStateSupplier(const GridMapStateSupplier<G>& other): AbstractSimpleWeightedDirectedGraphStateSupplier<GridMapState, G, xyLoc, cost_t>{other} {

        }
        GridMapStateSupplier(const GridMapStateSupplier<G>&& other): AbstractSimpleWeightedDirectedGraphStateSupplier<GridMapState, G, xyLoc, cost_t>{::std::move(other)} {
            
        }
        virtual ~GridMapStateSupplier() {

        }
        GridMapStateSupplier& operator =(const GridMapStateSupplier<G>& other) {
            AbstractSimpleWeightedDirectedGraphStateSupplier<GridMapState, G, xyLoc, cost_t>::operator=(other);
            return *this;
        }
        GridMapStateSupplier& operator =(GridMapStateSupplier<G>&& other) {
            AbstractSimpleWeightedDirectedGraphStateSupplier<GridMapState, G, xyLoc, cost_t>::operator=(other);
            return *this;
        }
    };

    /**
     * @brief an expander that uses an underlying graph to compute successors
     * 
     * The successors of a state are the successors of the vertex representing the state
     * 
     * @note
     * Note that the state id of a search state is **the same** of the vertex id of a node. This
     * means that the state space is as large as the underlying graph!
     * 
     * @tparam G paylaod of the whole graph
     * @tparam V paylaod of each single vertex
     */
    template <typename G>
    class SimpleGridMapStateExpander: public IStateExpander<GridMapState, nodeid_t> {
    private:
        const cpp_utils::graphs::IImmutableGraph<G, xyLoc, cost_t>& graph;
    public:
        SimpleGridMapStateExpander(const cpp_utils::graphs::IImmutableGraph<G, xyLoc, cost_t>& graph): graph{graph} {

        }
        virtual ~SimpleGridMapStateExpander() {

        }
        SimpleGridMapStateExpander(const SimpleGridMapStateExpander<G>& other) : graph{other.graph} {

        }
        SimpleGridMapStateExpander<G>& operator =(const SimpleGridMapStateExpander<G>& other) {
            this->graph = other.graph;
            return *this;
        }
        SimpleGridMapStateExpander(SimpleGridMapStateExpander<G>&& other): graph{other.graph} {

        }
        SimpleGridMapStateExpander<G>& operator =(SimpleGridMapStateExpander<G>&& other) {
            this->graph = other.graph;
            return *this;
        }
    public:
        virtual cpp_utils::vectorplus<std::pair<GridMapState&, cost_t>> getSuccessors(const GridMapState& state, IStateSupplier<GridMapState, nodeid_t>& supplier) {
            cpp_utils::vectorplus<std::pair<GridMapState&, cost_t>> result{};
            for (auto outEdge : graph.getOutEdges(state.getId())) {
                fine("an outedge ", outEdge, " of ", state, "(", &state, ") goes to", outEdge.getSinkId(), "edge payload of", outEdge.getPayload());
                result.add(std::pair<GridMapState&, cost_t>{
                    supplier.getState(outEdge.getSinkId()),
                    outEdge.getPayload()
                });
            }
            return result;
        }
        virtual std::pair<GridMapState&, cost_t> getSuccessor(const GridMapState& state, int successorNumber, IStateSupplier<GridMapState, nodeid_t>& supplier) {
            auto outEdge = this->graph.getOutEdge(state.getId(), static_cast<moveid_t>(successorNumber));
            return std::pair<GridMapState&, cost_t>{supplier.getState(outEdge.getSinkId()), outEdge.getPayload()};
        }
    public:
        virtual void cleanup() {

        }
    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            return MemoryConsumption{sizeof(*this), MemoryConsumptionEnum::BYTE};
        }
    };

    class SAPFGridMapGoalChecker: public IGoalChecker<GridMapState> {
    public:
        virtual bool isGoal(const GridMapState& state, const GridMapState* goal) const {
            return state.getPosition() == goal->getPosition();
        }
    public:
        virtual void cleanup() {

        }
    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            return MemoryConsumption{sizeof(*this), MemoryConsumptionEnum::BYTE};
        }
    };



}

#endif 